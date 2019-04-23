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
ULOG_DECLARE_TAG(vbuf);


int vbuf_new(size_t capacity,
	     size_t userdata_capacity,
	     const struct vbuf_cbs *cbs,
	     struct vbuf_pool *pool,
	     struct vbuf_buffer **ret_obj)
{
	int res = 0, err, mutex_init = 0;
	struct vbuf_buffer *buf;

	ULOG_ERRNO_RETURN_ERR_IF(cbs == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs->alloc == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs->free == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == NULL, EINVAL);

	buf = calloc(1, sizeof(*buf));
	if (buf == NULL) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc:buf", -res);
		*ret_obj = NULL;
		return -res;
	}

	list_node_unref(&buf->node);
	buf->capacity = capacity;
	list_init(&buf->metas);
	buf->userdata_capacity = userdata_capacity;
	buf->cbs = *cbs;
	buf->pool = pool;

	res = pthread_mutex_init(&buf->mutex, NULL);
	if (res != 0) {
		res = -res;
		ULOG_ERRNO("pthread_mutex_init", -res);
		goto error;
	}
	mutex_init = 1;

	/* Video frame */
	res = (*buf->cbs.alloc)(buf, buf->cbs.alloc_userdata);
	if (res < 0) {
		ULOG_ERRNO("buf->alloc", -res);
		goto error;
	}

	/* User data */
	if (buf->userdata_capacity > 0) {
		buf->userdata_ptr = calloc(1, buf->userdata_capacity);
		if (buf->userdata_ptr == NULL) {
			res = -ENOMEM;
			ULOG_ERRNO("calloc:userdata", -res);
			goto error;
		}
	}

	vbuf_ref(buf);

	*ret_obj = buf;
	return 0;

error:
	if (mutex_init)
		pthread_mutex_destroy(&buf->mutex);
	err = (*buf->cbs.free)(buf, buf->cbs.free_userdata);
	if (err < 0)
		ULOG_ERRNO("buf->free", -err);
	free(buf->userdata_ptr);
	free(buf);
	*ret_obj = NULL;
	return res;
}


int vbuf_destroy(struct vbuf_buffer *buf)
{
	int res, ref;
	struct vbuf_meta *meta = NULL, *tmp_meta = NULL;

	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);

	ref = vbuf_get_ref_count(buf);
	if (ref > 0)
		ULOGW("ref count is not null! (%d)", ref);

	/* Video frame */
	res = (*buf->cbs.free)(buf, buf->cbs.free_userdata);
	if (res < 0)
		ULOG_ERRNO("buf->free", -res);

	/* Remove all metadata */
	list_walk_entry_forward_safe(&buf->metas, meta, tmp_meta, node)
	{
		list_del(&meta->node);
		vbuf_meta_destroy(meta);
	}

	/* User data */
	free(buf->userdata_ptr);
	buf->userdata_ptr = NULL;

	pthread_mutex_destroy(&buf->mutex);
	free(buf);

	return 0;
}


int vbuf_ref(struct vbuf_buffer *buf)
{
	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);

#if defined(__GNUC__)
	__atomic_add_fetch(&buf->ref_count, 1, __ATOMIC_SEQ_CST);
#else
#	error no atomic increment function found on this platform
#endif

	return 0;
}


int vbuf_unref(struct vbuf_buffer *buf)
{
	int ref = 0;
	int res = 0;

	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(!vbuf_is_ref(buf), ENOENT);

#if defined(__GNUC__)
	ref = __atomic_sub_fetch(&buf->ref_count, 1, __ATOMIC_SEQ_CST);
#else
#	error no atomic decrement function found on this platform
#endif

	if (ref == 0) {
		/* Call the callback function if implemented */
		if (buf->cbs.unref) {
			res = (*buf->cbs.unref)(buf, buf->cbs.unref_userdata);
			if (res < 0) {
				ULOG_ERRNO("buf->unref", -res);
				goto out;
			}
		}

		buf->write_locked = 0;
		buf->size = 0;

		if (buf->pool) {
			res = vbuf_pool_put(buf->pool, buf);
			if (res < 0)
				goto out;
		} else {
			res = vbuf_destroy(buf);
			if (res < 0)
				goto out;
		}
	}

out:
	return res;
}


int vbuf_is_ref(struct vbuf_buffer *buf)
{
	int ref_count = vbuf_get_ref_count(buf);

	if (ref_count < 0)
		return ref_count;
	else
		return (ref_count > 0) ? 1 : 0;
}


int vbuf_get_ref_count(struct vbuf_buffer *buf)
{
	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);

#if defined(__GNUC__)
	int ref_count = __atomic_load_n(&buf->ref_count, __ATOMIC_ACQUIRE);
#else
#	error no atomic load function found on this platform
#endif

	return ref_count;
}


int vbuf_write_lock(struct vbuf_buffer *buf)
{
	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);
#if defined(__GNUC__)
	int ref_count = __atomic_load_n(&buf->ref_count, __ATOMIC_ACQUIRE);
#else
#	error no atomic load function found on this platform
#endif
	ULOG_ERRNO_RETURN_ERR_IF(ref_count != 1, EBUSY);

	buf->write_locked = 1;
	return 0;
}


int vbuf_write_unlock(struct vbuf_buffer *buf)
{
	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);
#if defined(__GNUC__)
	int ref_count = __atomic_load_n(&buf->ref_count, __ATOMIC_ACQUIRE);
#else
#	error no atomic load function found on this platform
#endif
	ULOG_ERRNO_RETURN_ERR_IF(ref_count != 1, EBUSY);

	buf->write_locked = 0;
	return 0;
}


int vbuf_is_write_locked(struct vbuf_buffer *buf)
{
	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);

	return buf->write_locked;
}


struct vbuf_pool *vbuf_get_pool(struct vbuf_buffer *buf)
{
	ULOG_ERRNO_RETURN_VAL_IF(buf == NULL, EINVAL, NULL);

	return buf->pool;
}


uint8_t *vbuf_get_data(struct vbuf_buffer *buf)
{
	ULOG_ERRNO_RETURN_VAL_IF(buf == NULL, EINVAL, NULL);
	ULOG_ERRNO_RETURN_VAL_IF(buf->write_locked, EPERM, NULL);

	return buf->ptr;
}


const uint8_t *vbuf_get_cdata(struct vbuf_buffer *buf)
{
	ULOG_ERRNO_RETURN_VAL_IF(buf == NULL, EINVAL, NULL);

	return buf->ptr;
}


ssize_t vbuf_get_capacity(struct vbuf_buffer *buf)
{
	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);

	return (ssize_t)buf->capacity;
}


ssize_t vbuf_set_capacity(struct vbuf_buffer *buf, size_t capacity)
{
	size_t old_capacity;
	int res;

	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(buf->cbs.realloc == NULL, ENOSYS);
	ULOG_ERRNO_RETURN_ERR_IF(buf->write_locked, EPERM);

	old_capacity = buf->capacity;

	if (capacity > buf->capacity) {
		buf->capacity = capacity;
		res = (*buf->cbs.realloc)(buf, buf->cbs.realloc_userdata);
		if (res < 0) {
			buf->capacity = old_capacity;
			ULOG_ERRNO("buf->realloc", -res);
			return res;
		}
	}

	return (ssize_t)buf->capacity;
}


ssize_t vbuf_get_size(struct vbuf_buffer *buf)
{
	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);

	return (ssize_t)buf->size;
}


int vbuf_set_size(struct vbuf_buffer *buf, size_t size)
{
	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(size > buf->capacity, ENOBUFS);
	ULOG_ERRNO_RETURN_ERR_IF(buf->write_locked, EPERM);

	buf->size = size;

	return 0;
}


int vbuf_copy(struct vbuf_buffer *src_buf, struct vbuf_buffer *dst_buf)
{
	int res;

	ULOG_ERRNO_RETURN_ERR_IF(src_buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(dst_buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(dst_buf == src_buf, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(dst_buf->write_locked, EPERM);

	if (dst_buf->capacity < src_buf->size) {
		res = vbuf_set_capacity(dst_buf, src_buf->size);
		if (res < 0)
			return res;
	}
	if (src_buf->size > 0) {
		memcpy(dst_buf->ptr, src_buf->ptr, src_buf->size);
		res = vbuf_set_size(dst_buf, src_buf->size);
		if (res < 0)
			return res;
	}

	res = vbuf_userdata_copy(src_buf, dst_buf);
	if (res < 0)
		return res;

	res = vbuf_metadata_copy(src_buf, dst_buf, 0);
	if (res < 0)
		return res;

	return 0;
}


struct vbuf_meta *vbuf_meta_new(void *key, unsigned int level, size_t len)
{
	struct vbuf_meta *meta;

	ULOG_ERRNO_RETURN_VAL_IF(key == NULL, EINVAL, NULL);
	ULOG_ERRNO_RETURN_VAL_IF(len == 0, EINVAL, NULL);

	meta = calloc(1, sizeof(*meta));
	if (meta == NULL) {
		ULOG_ERRNO("calloc:meta", ENOMEM);
		return NULL;
	}
	list_node_unref(&meta->node);
	meta->key = key;
	meta->level = level;
	meta->len = len;
	meta->data = calloc(1, len);
	if (meta->data == NULL) {
		free(meta);
		ULOG_ERRNO("calloc:data", ENOMEM);
		return NULL;
	}

	return meta;
}


int vbuf_meta_destroy(struct vbuf_meta *meta)
{
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(!list_node_is_unref(&meta->node), EBUSY);

	free(meta->data);
	free(meta);

	return 0;
}


struct vbuf_meta *vbuf_meta_find(struct vbuf_buffer *buf, void *key)
{
	int found = 0;
	struct vbuf_meta *meta = NULL;

	ULOG_ERRNO_RETURN_VAL_IF(buf == NULL, EINVAL, NULL);
	ULOG_ERRNO_RETURN_VAL_IF(key == NULL, EINVAL, NULL);

	list_walk_entry_forward(&buf->metas, meta, node)
	{
		if (meta->key == key) {
			found = 1;
			break;
		}
	}

	return (found) ? meta : NULL;
}


int vbuf_metadata_add(struct vbuf_buffer *buf,
		      void *key,
		      unsigned int level,
		      size_t len,
		      uint8_t **ret_ptr)
{
	int res;
	struct vbuf_meta *meta;

	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(key == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_ptr == NULL, EINVAL);

	VBUF_MUTEX_LOCK(&buf->mutex);

	/* Check that the key does not already exist */
	meta = vbuf_meta_find(buf, key);
	if (meta != NULL) {
		VBUF_MUTEX_UNLOCK(&buf->mutex);
		res = -EEXIST;
		ULOG_ERRNO("metadata %p already exists", -res, key);
		return res;
	}

	meta = vbuf_meta_new(key, level, len);
	if (meta == NULL) {
		VBUF_MUTEX_UNLOCK(&buf->mutex);
		return -ENOMEM;
	}

	list_add_before(&buf->metas, &meta->node);

	VBUF_MUTEX_UNLOCK(&buf->mutex);

	*ret_ptr = meta->data;
	return 0;
}


int vbuf_metadata_get(struct vbuf_buffer *buf,
		      void *key,
		      unsigned int *level,
		      size_t *len,
		      uint8_t **ret_ptr)
{
	int res;
	struct vbuf_meta *meta;

	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(key == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_ptr == NULL, EINVAL);

	VBUF_MUTEX_LOCK(&buf->mutex);

	meta = vbuf_meta_find(buf, key);

	VBUF_MUTEX_UNLOCK(&buf->mutex);

	if (meta == NULL) {
		res = -ENOENT;
		ULOG_ERRNO("metadata %p not found", -res, key);
		return res;
	}

	if (level)
		*level = meta->level;
	if (len)
		*len = meta->len;
	*ret_ptr = meta->data;

	return 0;
}


int vbuf_metadata_remove(struct vbuf_buffer *buf, void *key)
{
	int res;
	struct vbuf_meta *meta;

	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(key == NULL, EINVAL);

	VBUF_MUTEX_LOCK(&buf->mutex);

	meta = vbuf_meta_find(buf, key);
	if (meta == NULL) {
		VBUF_MUTEX_UNLOCK(&buf->mutex);
		res = -ENOENT;
		ULOG_ERRNO("metadata %p not found", -res, key);
		return res;
	}

	list_del(&meta->node);

	VBUF_MUTEX_UNLOCK(&buf->mutex);

	vbuf_meta_destroy(meta);

	return 0;
}


int vbuf_metadata_copy(struct vbuf_buffer *src_buf,
		       struct vbuf_buffer *dst_buf,
		       unsigned int max_level)
{
	int res;
	struct vbuf_meta *src_meta;
	uint8_t *dst_meta;

	ULOG_ERRNO_RETURN_ERR_IF(src_buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(dst_buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(dst_buf == src_buf, EINVAL);

	VBUF_MUTEX_LOCK(&src_buf->mutex);

	list_walk_entry_forward(&src_buf->metas, src_meta, node)
	{
		if ((max_level == 0) || (src_meta->level < max_level)) {
			res = vbuf_metadata_add(dst_buf,
						src_meta->key,
						src_meta->level,
						src_meta->len,
						&dst_meta);
			if (res < 0) {
				VBUF_MUTEX_UNLOCK(&src_buf->mutex);
				ULOG_ERRNO("vbuf_metadata_add", -res);
				return res;
			}
			memcpy(dst_meta, src_meta->data, src_meta->len);
		}
	}

	VBUF_MUTEX_UNLOCK(&src_buf->mutex);

	return 0;
}


uint8_t *vbuf_get_userdata(struct vbuf_buffer *buf)
{
	ULOG_ERRNO_RETURN_VAL_IF(buf == NULL, EINVAL, NULL);
	ULOG_ERRNO_RETURN_VAL_IF(buf->write_locked, EPERM, NULL);

	return buf->userdata_ptr;
}


const uint8_t *vbuf_get_cuserdata(struct vbuf_buffer *buf)
{
	ULOG_ERRNO_RETURN_VAL_IF(buf == NULL, EINVAL, NULL);

	return buf->userdata_ptr;
}


ssize_t vbuf_get_userdata_capacity(struct vbuf_buffer *buf)
{
	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);

	return (ssize_t)buf->userdata_capacity;
}


ssize_t vbuf_set_userdata_capacity(struct vbuf_buffer *buf, size_t capacity)
{
	int res;

	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(buf->write_locked, EPERM);

	if (capacity > buf->userdata_capacity) {
		uint8_t *tmp = realloc(buf->userdata_ptr, capacity);
		if (tmp == NULL) {
			res = -ENOMEM;
			ULOG_ERRNO("calloc", -res);
			return res;
		}

		buf->userdata_ptr = tmp;
		buf->userdata_capacity = capacity;
	}

	return (ssize_t)buf->userdata_capacity;
}


ssize_t vbuf_get_userdata_size(struct vbuf_buffer *buf)
{
	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);

	return (ssize_t)buf->userdata_size;
}


int vbuf_set_userdata_size(struct vbuf_buffer *buf, size_t size)
{
	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(size > buf->userdata_capacity, ENOBUFS);
	ULOG_ERRNO_RETURN_ERR_IF(buf->write_locked, EPERM);

	buf->userdata_size = size;

	return 0;
}


int vbuf_userdata_copy(struct vbuf_buffer *src_buf, struct vbuf_buffer *dst_buf)
{
	int res;

	ULOG_ERRNO_RETURN_ERR_IF(src_buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(dst_buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(dst_buf == src_buf, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(dst_buf->write_locked, EPERM);

	if (src_buf->userdata_size == 0)
		return 0;

	res = vbuf_set_userdata_capacity(dst_buf, src_buf->userdata_capacity);
	if (res < 0)
		return res;

	memcpy(dst_buf->userdata_ptr,
	       src_buf->userdata_ptr,
	       src_buf->userdata_size);
	res = vbuf_set_userdata_size(dst_buf, src_buf->userdata_size);
	return res;
}
