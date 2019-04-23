/**
 * Copyright (c) 2017 Parrot Drones SAS
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the Parrot Drones SAS Company nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE PARROT DRONES SAS COMPANY BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "vbuf_priv.h"


int vbuf_queue_new(unsigned int max_count,
		   int drop_when_full,
		   struct vbuf_queue **ret_obj)
{
	int res = 0, mutex_init = 0, cond_init = 0;

	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == NULL, EINVAL);

	struct vbuf_queue *queue = calloc(1, sizeof(*queue));
	if (queue == NULL) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc:queue", -res);
		*ret_obj = NULL;
		return res;
	}

	list_init(&queue->buffers);
	queue->max_count = max_count;
	queue->drop_when_full = drop_when_full;

	res = pthread_mutex_init(&queue->mutex, NULL);
	if (res != 0) {
		res = -res;
		ULOG_ERRNO("pthread_mutex_init", -res);
		goto error;
	}
	mutex_init = 1;

	res = pthread_cond_init(&queue->cond, NULL);
	if (res != 0) {
		res = -res;
		ULOG_ERRNO("pthread_cond_init", -res);
		goto error;
	}
	cond_init = 1;

	queue->evt = pomp_evt_new();
	if (queue->evt == NULL) {
		res = -ENOMEM;
		ULOG_ERRNO("pomp_evt_new", -res);
		goto error;
	}

	*ret_obj = queue;
	return 0;

error:
	if (mutex_init)
		pthread_mutex_destroy(&queue->mutex);
	if (cond_init)
		pthread_cond_destroy(&queue->cond);
	if (queue->evt != NULL)
		pomp_evt_destroy(queue->evt);
	free(queue);
	*ret_obj = NULL;
	return res;
}


int vbuf_queue_destroy(struct vbuf_queue *queue)
{
	if (queue == NULL)
		return 0;

	VBUF_MUTEX_LOCK(&queue->mutex);
	if (queue->count != 0) {
		ULOGW("destroying queue but it is not empty! "
		      "flushing %d buffers...",
		      queue->count);
	}
	VBUF_MUTEX_UNLOCK(&queue->mutex);

	vbuf_queue_flush(queue);

	pthread_mutex_destroy(&queue->mutex);
	pthread_cond_destroy(&queue->cond);
	pomp_evt_destroy(queue->evt);
	free(queue);

	return 0;
}


int vbuf_queue_get_count(struct vbuf_queue *queue)
{
	int count;

	ULOG_ERRNO_RETURN_ERR_IF(queue == NULL, EINVAL);

	VBUF_MUTEX_LOCK(&queue->mutex);
	count = queue->count;
	VBUF_MUTEX_UNLOCK(&queue->mutex);

	return count;
}


int vbuf_queue_peek(struct vbuf_queue *queue,
		    unsigned int index,
		    int timeout_ms,
		    struct vbuf_buffer **buf)
{
	int err = 0, res = 0, found = 0;
	unsigned int idx = 0;
	struct timespec ts;
	struct vbuf_queue_buffer *qb = NULL;

	ULOG_ERRNO_RETURN_ERR_IF(queue == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);

	VBUF_MUTEX_LOCK(&queue->mutex);

	if (queue->count < index + 1) {
		if (timeout_ms > 0) {
			/* Wait until timeout */
			vbuf_get_time_with_ms_delay(&ts, timeout_ms);
			err = pthread_cond_timedwait(
				&queue->cond, &queue->mutex, &ts);
		} else if (timeout_ms == 0) {
			/* No wait, return */
			res = -EAGAIN;
			goto out;
		} else {
			/* Wait forever */
			err = pthread_cond_wait(&queue->cond, &queue->mutex);
		}
		if (err == ETIMEDOUT) {
			/* Timeout */
			res = -ETIMEDOUT;
			goto out;
		} else if (err != 0) {
			/* Other error */
			ULOG_ERRNO("pthread_cond_wait", err);
			res = -err;
			goto out;
		}
	}

	if (queue->count < index + 1) {
		/* Still no buffer after waiting */
		res = -EAGAIN;
		goto out;
	}

	/* A buffer is available */
	list_walk_entry_forward(&queue->buffers, qb, node)
	{
		if (idx == index) {
			found = 1;
			break;
		}
		idx++;
	}
	if (!found) {
		res = -EAGAIN;
		goto out;
	}

	/* Call the callback function if implemented */
	if (qb->buffer->cbs.queue_peek) {
		VBUF_MUTEX_UNLOCK(&queue->mutex);

		res = (*qb->buffer->cbs.queue_peek)(
			qb->buffer,
			timeout_ms,
			qb->buffer->cbs.queue_peek_userdata);

		goto out2;
	}

out:
	VBUF_MUTEX_UNLOCK(&queue->mutex);

out2:
	*buf = ((res == 0) && (qb)) ? qb->buffer : NULL;

	return res;
}


int vbuf_queue_pop(struct vbuf_queue *queue,
		   int timeout_ms,
		   struct vbuf_buffer **buf)
{
	int err = 0, res = 0;
	struct timespec ts;
	struct vbuf_queue_buffer *qb = NULL;

	ULOG_ERRNO_RETURN_ERR_IF(queue == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);

	VBUF_MUTEX_LOCK(&queue->mutex);

	if (queue->count == 0) {
		if (timeout_ms > 0) {
			/* Wait until timeout */
			vbuf_get_time_with_ms_delay(&ts, timeout_ms);
			err = pthread_cond_timedwait(
				&queue->cond, &queue->mutex, &ts);
		} else if (timeout_ms == 0) {
			/* No wait, return */
			res = -EAGAIN;
			goto out;
		} else {
			/* Wait forever */
			err = pthread_cond_wait(&queue->cond, &queue->mutex);
		}
		if (err == ETIMEDOUT) {
			/* Timeout */
			res = -ETIMEDOUT;
			goto out;
		} else if (err != 0) {
			/* Other error */
			ULOG_ERRNO("pthread_cond_wait", err);
			res = -err;
			goto out;
		}
	}

	if (queue->count == 0) {
		/* Still no buffer after waiting */
		res = -EAGAIN;
		goto out;
	}

	/* A buffer is available */
	qb = list_entry(list_first(&queue->buffers), typeof(*qb), node);

	/* Remove the buffer from the list */
	list_del(&qb->node);
	queue->count--;

	/* Call the callback function if implemented */
	if (qb->buffer->cbs.queue_pop) {
		VBUF_MUTEX_UNLOCK(&queue->mutex);

		res = (*qb->buffer->cbs.queue_pop)(
			qb->buffer,
			timeout_ms,
			qb->buffer->cbs.queue_pop_userdata);
		if (res < 0)
			vbuf_unref(qb->buffer);

		goto out2;
	}

out:
	VBUF_MUTEX_UNLOCK(&queue->mutex);

out2:
	*buf = ((res == 0) && (qb)) ? qb->buffer : NULL;
	free(qb);

	return res;
}


int vbuf_queue_push(struct vbuf_queue *queue, struct vbuf_buffer *buf)
{
	int res = 0;
	struct vbuf_buffer *_buf = NULL;
	struct vbuf_queue_buffer *qb = NULL;

	ULOG_ERRNO_RETURN_ERR_IF(queue == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);

	VBUF_MUTEX_LOCK(&queue->mutex);

	/* Finite queue */
	if ((queue->max_count > 0) && (queue->count == queue->max_count)) {
		/* The queue is full */
		if (queue->drop_when_full) {
			/* Drop the oldest buffer */
			VBUF_MUTEX_UNLOCK(&queue->mutex);
			res = vbuf_queue_pop(queue, 0, &_buf);
			if ((res < 0) && (res != -EAGAIN)) {
				ULOG_ERRNO("vbuf_queue_pop", -res);
			} else if (_buf != NULL) {
				res = vbuf_unref(_buf);
				if (res < 0)
					ULOG_ERRNO("vbuf_unref", -res);
			}
			VBUF_MUTEX_LOCK(&queue->mutex);
		} else {
			VBUF_MUTEX_UNLOCK(&queue->mutex);
			return -EAGAIN;
		}
	}

	/* Call the callback function if implemented */
	if (buf->cbs.queue_push) {
		VBUF_MUTEX_UNLOCK(&queue->mutex);
		res = (*buf->cbs.queue_push)(buf, buf->cbs.queue_push_userdata);
		if (res < 0)
			return res;
		VBUF_MUTEX_LOCK(&queue->mutex);
	}

	/* Create the buffer queue element */
	qb = calloc(1, sizeof(*qb));
	if (qb == NULL) {
		VBUF_MUTEX_UNLOCK(&queue->mutex);
		res = -ENOMEM;
		ULOG_ERRNO("calloc:queue_buf", -res);
		return res;
	}
	list_node_unref(&qb->node);
	qb->buffer = buf;
	vbuf_ref(buf);

	/* Add the buffer to the list */
	list_add_after(list_last(&queue->buffers), &qb->node);
	queue->count++;

	/* Notify that a buffer available */
	res = pomp_evt_signal(queue->evt);
	if (res < 0)
		ULOG_ERRNO("pomp_evt_signal", -res);

	if (queue->count == 1) {
		/* The queue was empty,
		 * someone might been waiting for a buffer */
		VBUF_COND_SIGNAL(&queue->cond);
	}

	VBUF_MUTEX_UNLOCK(&queue->mutex);

	return 0;
}


int vbuf_queue_abort(struct vbuf_queue *queue)
{
	ULOG_ERRNO_RETURN_ERR_IF(queue == NULL, EINVAL);

	VBUF_COND_BROADCAST(&queue->cond);

	return 0;
}


int vbuf_queue_flush(struct vbuf_queue *queue)
{
	struct vbuf_queue_buffer *qb = NULL, *tmp_qb = NULL;

	ULOG_ERRNO_RETURN_ERR_IF(queue == NULL, EINVAL);

	VBUF_MUTEX_LOCK(&queue->mutex);

	if (queue->count == 0) {
		/* The queue is already empty */
		VBUF_MUTEX_UNLOCK(&queue->mutex);
		return 0;
	}

	/* Dequeue all buffers */
	list_walk_entry_forward_safe(&queue->buffers, qb, tmp_qb, node)
	{
		list_del(&qb->node);
		queue->count--;
		vbuf_unref(qb->buffer);
		free(qb);
	}

	VBUF_MUTEX_UNLOCK(&queue->mutex);

	return 0;
}


struct pomp_evt *vbuf_queue_get_evt(struct vbuf_queue *queue)
{
	ULOG_ERRNO_RETURN_VAL_IF(queue == NULL, EINVAL, NULL);

	return queue->evt;
}
