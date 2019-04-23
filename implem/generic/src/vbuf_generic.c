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

#include <errno.h>
#include <unistd.h>

#define ULOG_TAG vbuf_generic
#include <ulog.h>
ULOG_DECLARE_TAG(vbuf_generic);

#include <video-buffers/vbuf_generic.h>
#include <video-buffers/vbuf_private.h>


#define VBUF_TYPE_GENERIC 0x56425546 /* "VBUF" */


struct vbuf_generic {
	/* nothing to do here */
	void *dummy;
};


static int vbuf_generic_alloc_cb(struct vbuf_buffer *buf, void *userdata)
{
	int res = 0;

	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);

	buf->type = VBUF_TYPE_GENERIC;

	buf->specific = calloc(1, sizeof(struct vbuf_generic));
	if (buf->specific == NULL)
		return -ENOMEM;
	if (buf->capacity > 0) {
		buf->ptr = calloc(1, buf->capacity);
		if (buf->ptr == NULL) {
			res = -ENOMEM;
			goto error;
		}
	}

	return 0;

error:
	free(buf->specific);
	return res;
}


static int vbuf_generic_realloc_cb(struct vbuf_buffer *buf, void *userdata)
{
	int res = 0;

	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);

	if (buf->capacity > 0) {
		uint8_t *tmp = realloc(buf->ptr, buf->capacity);
		if (tmp == NULL)
			return -ENOMEM;
		buf->ptr = tmp;
	}

	return res;
}


static int vbuf_generic_free_cb(struct vbuf_buffer *buf, void *userdata)
{
	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);

	free(buf->ptr);
	buf->ptr = NULL;
	free(buf->specific);
	buf->specific = NULL;

	return 0;
}


int vbuf_generic_get_cbs(struct vbuf_cbs *cbs)
{
	ULOG_ERRNO_RETURN_ERR_IF(cbs == NULL, EINVAL);

	memset(cbs, 0, sizeof(*cbs));
	cbs->alloc = vbuf_generic_alloc_cb;
	cbs->alloc_userdata = NULL;
	cbs->realloc = vbuf_generic_realloc_cb;
	cbs->realloc_userdata = NULL;
	cbs->free = vbuf_generic_free_cb;
	cbs->free_userdata = NULL;

	return 0;
}
