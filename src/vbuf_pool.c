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


int vbuf_pool_new(unsigned int count,
		  size_t capacity,
		  size_t userdata_capacity,
		  const struct vbuf_cbs *cbs,
		  struct vbuf_pool **ret_obj)
{
	int res = 0, mutex_init = 0, cond_init = 0;
	unsigned int i;
	struct vbuf_buffer *buf = NULL, *tmp_buf;
	struct vbuf_pool *pool;

	ULOG_ERRNO_RETURN_ERR_IF(count == 0, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == NULL, EINVAL);

	pool = calloc(1, sizeof(*pool));
	if (pool == NULL) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc:pool", -res);
		*ret_obj = NULL;
		return res;
	}

	pool->count = count;
	list_init(&pool->buffers);

	res = pthread_mutex_init(&pool->mutex, NULL);
	if (res != 0) {
		res = -res;
		ULOG_ERRNO("pthread_mutex_init", -res);
		goto error;
	}
	mutex_init = 1;

	res = pthread_cond_init(&pool->cond, NULL);
	if (res != 0) {
		res = -res;
		ULOG_ERRNO("pthread_cond_init", -res);
		goto error;
	}
	cond_init = 1;

	pool->evt = pomp_evt_new();
	if (pool->evt == NULL) {
		res = -ENOMEM;
		ULOG_ERRNO("pomp_evt_new", -res);
		goto error;
	}

	/* Allocate all buffers */
	for (i = 0; i < pool->count; i++) {
		res = vbuf_new(capacity, userdata_capacity, cbs, pool, &buf);
		if (res < 0)
			goto error;

		vbuf_unref(buf);
		buf = NULL;
	}

	*ret_obj = pool;
	return 0;

error:
	buf = NULL;
	tmp_buf = NULL;
	list_walk_entry_forward_safe(&pool->buffers, buf, tmp_buf, node)
	{
		vbuf_destroy(buf);
	}

	if (mutex_init)
		pthread_mutex_destroy(&pool->mutex);
	if (cond_init)
		pthread_cond_destroy(&pool->cond);
	if (pool->evt != NULL)
		pomp_evt_destroy(pool->evt);
	free(pool);
	*ret_obj = NULL;
	return res;
}


int vbuf_pool_destroy(struct vbuf_pool *pool)
{
	struct vbuf_buffer *buf = NULL, *tmp_buf = NULL;

	if (pool == NULL)
		return 0;

	VBUF_MUTEX_LOCK(&pool->mutex);

	if (pool->free != pool->count) {
		ULOGW("not all buffers have been returned! (%d vs. %d)",
		      pool->free,
		      pool->count);
	}

	/* Free all buffers */
	list_walk_entry_forward_safe(&pool->buffers, buf, tmp_buf, node)
	{
		list_del(&buf->node);
		pool->free--;
		vbuf_destroy(buf);
	}

	VBUF_MUTEX_UNLOCK(&pool->mutex);

	pthread_mutex_destroy(&pool->mutex);
	pthread_cond_destroy(&pool->cond);
	pomp_evt_destroy(pool->evt);
	free(pool);

	return 0;
}


int vbuf_pool_get_count(struct vbuf_pool *pool)
{
	int count;

	ULOG_ERRNO_RETURN_ERR_IF(pool == NULL, EINVAL);

	VBUF_MUTEX_LOCK(&pool->mutex);
	count = pool->free;
	VBUF_MUTEX_UNLOCK(&pool->mutex);

	return count;
}


int vbuf_pool_get(struct vbuf_pool *pool,
		  int timeout_ms,
		  struct vbuf_buffer **buf)
{
	int err = 0, res = 0;
	struct timespec ts;
	struct vbuf_buffer *_buf = NULL;

	ULOG_ERRNO_RETURN_ERR_IF(pool == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);

	VBUF_MUTEX_LOCK(&pool->mutex);

	if (pool->free == 0) {
		if (timeout_ms > 0) {
			/* Wait until timeout */
			vbuf_get_time_with_ms_delay(&ts, timeout_ms);
			err = pthread_cond_timedwait(
				&pool->cond, &pool->mutex, &ts);
		} else if (timeout_ms == 0) {
			/* No wait, return */
			res = -EAGAIN;
			goto out;
		} else {
			/* Wait forever */
			err = pthread_cond_wait(&pool->cond, &pool->mutex);
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

	if (pool->free == 0) {
		/* Still no buffer after waiting */
		res = -EAGAIN;
		goto out;
	}

	/* A buffer is available */
	_buf = list_entry(list_first(&pool->buffers), typeof(*_buf), node);

	vbuf_ref(_buf);

	/* Remove the buffer from the list */
	list_del(&_buf->node);
	pool->free--;

	/* Call the callback function if implemented */
	if (_buf->cbs.pool_get) {
		VBUF_MUTEX_UNLOCK(&pool->mutex);

		res = (*_buf->cbs.pool_get)(
			_buf, timeout_ms, _buf->cbs.pool_get_userdata);
		if (res < 0) {
			vbuf_unref(_buf);
			return res;
		}

		goto out2;
	}

out:
	VBUF_MUTEX_UNLOCK(&pool->mutex);

out2:
	*buf = _buf;

	return res;
}


int vbuf_pool_put(struct vbuf_pool *pool, struct vbuf_buffer *buf)
{
	int res = 0;
	struct vbuf_meta *meta = NULL, *tmp_meta = NULL;

	ULOG_ERRNO_RETURN_ERR_IF(pool == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(buf->pool == NULL, EINVAL);

	if (vbuf_get_ref_count(buf) > 0)
		ULOGW("ref count is not null! (%d)", buf->ref_count);

	/* Call the callback function if implemented */
	if (buf->cbs.pool_put) {
		res = (*buf->cbs.pool_put)(buf, buf->cbs.pool_put_userdata);
		if (res < 0)
			return res;
	}

	/* Remove all metadata */
	list_walk_entry_forward_safe(&buf->metas, meta, tmp_meta, node)
	{
		list_del(&meta->node);
		res = vbuf_meta_destroy(meta);
		if (res < 0)
			ULOG_ERRNO("vbuf_meta_destroy", -res);
	}

	VBUF_MUTEX_LOCK(&pool->mutex);

	/* Add the buffer to the list */
	list_add_after(list_last(&pool->buffers), &buf->node);
	pool->free++;

	/* Notify that a buffer is available */
	res = pomp_evt_signal(pool->evt);
	if (res < 0)
		ULOG_ERRNO("pomp_evt_signal", -res);

	if (pool->free == 1) {
		/* The pool was empty,
		 * someone might been waiting for a buffer */
		VBUF_COND_SIGNAL(&pool->cond);
	}

	VBUF_MUTEX_UNLOCK(&pool->mutex);

	return 0;
}


int vbuf_pool_abort(struct vbuf_pool *pool)
{
	ULOG_ERRNO_RETURN_ERR_IF(pool == NULL, EINVAL);

	VBUF_COND_BROADCAST(&pool->cond);

	return 0;
}


struct pomp_evt *vbuf_pool_get_evt(struct vbuf_pool *pool)
{
	ULOG_ERRNO_RETURN_VAL_IF(pool == NULL, EINVAL, NULL);

	return pool->evt;
}
