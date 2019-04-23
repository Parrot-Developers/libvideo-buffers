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

#ifndef _VBUF_PRIV_H_
#define _VBUF_PRIV_H_

#include <errno.h>
#include <pthread.h>
#include <unistd.h>

#define ULOG_TAG vbuf
#include <ulog.h>

#include <futils/futils.h>
#include <libpomp.h>
#include <video-buffers/vbuf.h>
#include <video-buffers/vbuf_private.h>


/* Lock mutex and warn on error */
#define VBUF_MUTEX_LOCK(_mutex)                                                \
	do {                                                                   \
		int __ret = pthread_mutex_lock(_mutex);                        \
		if (__ret != 0)                                                \
			ULOG_ERRNO("pthread_mutex_lock", __ret);               \
	} while (0)

/* Unlock mutex and warn on error */
#define VBUF_MUTEX_UNLOCK(_mutex)                                              \
	do {                                                                   \
		int __ret = pthread_mutex_unlock(_mutex);                      \
		if (__ret != 0)                                                \
			ULOG_ERRNO("pthread_mutex_lock", __ret);               \
	} while (0)

/* Signal condition and warn on error */
#define VBUF_COND_SIGNAL(_cond)                                                \
	do {                                                                   \
		int __ret = pthread_cond_signal(_cond);                        \
		if (__ret != 0)                                                \
			ULOG_ERRNO("pthread_cond_signal", __ret);              \
	} while (0)

/* Broadcast condition and warn on error */
#define VBUF_COND_BROADCAST(_cond)                                             \
	do {                                                                   \
		int __ret = pthread_cond_broadcast(_cond);                     \
		if (__ret != 0)                                                \
			ULOG_ERRNO("pthread_cond_broadcast", __ret);           \
	} while (0)

/* Wait on condition and warn on error */
#define VBUF_COND_WAIT(_cond, _mutex)                                          \
	do {                                                                   \
		int __ret = pthread_cond_wait(_cond, _mutex);                  \
		if (__ret != 0)                                                \
			ULOG_ERRNO("pthread_cond_wait", __ret);                \
	} while (0)


struct vbuf_meta {
	void *key;
	unsigned int level;
	uint8_t *data;
	size_t len;
	struct list_node node;
};


struct vbuf_pool {
	unsigned int count;
	unsigned int free;
	struct list_node buffers;
	pthread_mutex_t mutex;
	pthread_cond_t cond;
	struct pomp_evt *evt;
};


struct vbuf_queue_buffer {
	struct vbuf_buffer *buffer;
	struct list_node node;
};


struct vbuf_queue {
	unsigned int count;
	unsigned int max_count;
	int drop_when_full;
	struct list_node buffers;
	pthread_mutex_t mutex;
	pthread_cond_t cond;
	struct pomp_evt *evt;
};


int vbuf_is_ref(struct vbuf_buffer *buf);


int vbuf_destroy(struct vbuf_buffer *buf);


int vbuf_pool_put(struct vbuf_pool *pool, struct vbuf_buffer *buf);


struct vbuf_meta *vbuf_meta_new(void *key, unsigned int level, size_t len);


int vbuf_meta_destroy(struct vbuf_meta *meta);


struct vbuf_meta *vbuf_meta_find(struct vbuf_buffer *buf, void *key);


static inline void vbuf_get_time_with_ms_delay(struct timespec *ts,
					       unsigned int delay)
{
	struct timeval tp;
	gettimeofday(&tp, NULL);

	struct timespec ts2;
	time_timeval_to_timespec(&tp, &ts2);
	time_timespec_add_us(&ts2, (int64_t)delay * 1000, ts);
}


#endif /* !_VBUF_PRIV_H_ */
