/*
 * This file is part of the OpenMV project.
 * Copyright (c) 2013-2016 Kwabena W. Agyeman <kwagyeman@openmv.io>
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 * File System Helper Functions
 *
 */
#include <mp.h>
#include "common.h"
#include "fb_alloc.h"
#include "ff_wrapper.h"
#define FF_MIN(x,y) (((x)<(y))?(x):(y))

#define FF_USE_MPY_DRIVER	0

NORETURN static void ff_fail(FIL *fp, FRESULT res)
{
    if (fp) dfs_file_close(fp);
    nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, ffs_strerror(res)));
}

NORETURN static void ff_read_fail(FIL *fp)
{
    if (fp) dfs_file_close(fp);
    nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "Failed to read requested bytes!"));
}

NORETURN static void ff_write_fail(FIL *fp)
{
    if (fp) dfs_file_close(fp);
    nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "Failed to write requested bytes!"));
}

NORETURN static void ff_expect_fail(FIL *fp)
{
    if (fp) dfs_file_close(fp);
    nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "Unexpected value read!"));
}

NORETURN void ff_unsupported_format(FIL *fp)
{
    if (fp) dfs_file_close(fp);
    nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "Unsupported format!"));
}

NORETURN void ff_file_corrupted(FIL *fp)
{
    if (fp) dfs_file_close(fp);
    nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "File corrupted!"));
}

NORETURN void ff_not_equal(FIL *fp)
{
    if (fp) dfs_file_close(fp);
    nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "Images not equal!"));
}

NORETURN void ff_no_intersection(FIL *fp)
{
    if (fp) dfs_file_close(fp);
    nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "No intersection!"));
}

void file_read_open(FIL *fp, const char *path)
{
    FRESULT res = f_open_helper(fp, path, FA_READ|FA_OPEN_EXISTING);
    if (res != FR_OK) nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, ffs_strerror(res)));
}

void file_write_open(FIL *fp, const char *path)
{
    FRESULT res = f_open_helper(fp, path, FA_WRITE|FA_CREATE_ALWAYS);
    if (res != FR_OK) nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, ffs_strerror(res)));
}

void file_close(FIL *fp)
{
    FRESULT res = dfs_file_close(fp);
    if (res != FR_OK) ff_fail(fp, res);
}

void file_seek(FIL *fp, UINT offset)
{
    UINT res = dfs_file_lseek(fp, offset);
    if (res != offset) ff_fail(fp, res);
}

void file_truncate(FIL *fp)
{
    int res = dfs_file_ftruncate(fp,0);
    if (res < 0) ff_fail(fp, res);
}

void file_sync(FIL *fp)
{
    int res = dfs_file_flush(fp);
    if (res < 0) ff_fail(fp, res);
}

// These wrapper functions are used for backward compatibility with
// OpenMV code using vanilla FatFS. Note: Extracted from cc3200 ftp.c
#if FF_USE_MPY_DRIVER	
STATIC FATFS *lookup_path(const TCHAR **path) {
	
    mp_vfs_mount_t *fs = mp_vfs_lookup_path(*path, path);
    if (fs == MP_VFS_NONE || fs == MP_VFS_ROOT) {
        return NULL;
    }
    // here we assume that the mounted device is FATFS
    return &((fs_user_mount_t*)MP_OBJ_TO_PTR(fs->obj))->fatfs;
	
}
#endif
int lookup_fullpath(const TCHAR *fname, vstr_t *fpath)
{
	size_t path_num;
    mp_obj_t *path_items;
    mp_obj_list_get(mp_sys_path, &path_num, &path_items);
	for (size_t i = 0; i < path_num; i++) {
            vstr_reset(fpath);
			memset(fpath->buf,0x00,fpath->alloc);
            size_t p_len;
            const char *p = mp_obj_str_get_data(path_items[i], &p_len);
            if (p_len > 0) {
                vstr_add_strn(fpath, p, p_len);
            }
            vstr_add_strn(fpath, fname, strlen(fname));
			fpath->buf[fpath->len+1] = 0x00;
            mp_import_stat_t stat = mp_import_stat(fpath->buf);
			if (stat == MP_IMPORT_STAT_FILE)
			{
				return FR_OK;
			}
        }
	return FR_NO_FILE;
}

FRESULT f_open_helper(FIL *fp, const TCHAR *path, BYTE mode) {
#if FF_USE_MPY_DRIVER		
    FATFS *fs = lookup_path(&path);
    if (fs == NULL) {
        return FR_NO_PATH;
    }
#endif
	if (fp != NULL)
		memset(fp,0x0,sizeof(FIL));
	
	if ((mode & (FA_READ|FA_OPEN_EXISTING)) == (FA_READ|FA_OPEN_EXISTING))
	{
		VSTR_FIXED(full_path, MICROPY_ALLOC_PATH_MAX)
		if (lookup_fullpath(path,&full_path) == FR_OK)
			return dfs_file_open(fp, full_path.buf, O_RDONLY);
	}
	else if ((mode & (FA_WRITE|FA_CREATE_ALWAYS)) == (FA_WRITE|FA_CREATE_ALWAYS))
	{
        if(strchr(path, '/') != 0)
        {
		    return dfs_file_open(fp, path, O_WRONLY|O_RDWR| O_CREAT);
        }
        else
        {
            VSTR_FIXED(full_path, MICROPY_ALLOC_PATH_MAX)
            if(rt_device_find("sd0") != RT_NULL)
            {
                vstr_add_strn(&full_path, "/sd/", strlen("/sd/"));
            }
            else
            {
                vstr_add_strn(&full_path, "/flash/", strlen("/flash/"));
            }
            vstr_add_strn(&full_path, path, strlen(path));
            full_path.buf[full_path.len+1] = 0x00;
            mp_printf(MP_PYTHON_PRINTER, "Openfile : %s!\r\n",full_path.buf);
            return dfs_file_open(fp, full_path.buf, O_WRONLY | O_CREAT);
        }
	}
	
	return FR_NO_FILE;
}

//FRESULT f_opendir_helper(FF_DIR *dp, const TCHAR *path) {
//#if FF_USE_MPY_DRIVER		
//    FATFS *fs = lookup_path(&path);
//    if (fs == NULL) {
//        return FR_NO_PATH;
//    }
//#endif	
//    return f_opendir(dp, path);
//}

FRESULT f_stat_helper(const TCHAR *path, FILINFO *fno) {
#if FF_USE_MPY_DRIVER		
    FATFS *fs = lookup_path(&path);
    if (fs == NULL) {
        return FR_NO_PATH;
    }
#endif	
    //return dfs_file_stat(path, fno);
	return FR_NOT_READY;
}

FRESULT f_mkdir_helper(const TCHAR *path) {
#if FF_USE_MPY_DRIVER		
    FATFS *fs = lookup_path(&path);
    if (fs == NULL) {
        return FR_NO_PATH;
    }
#endif	
    //return f_mkdir(path);
	return FR_NOT_READY;
}

FRESULT f_unlink_helper(const TCHAR *path) {
#if FF_USE_MPY_DRIVER		
    FATFS *fs = lookup_path(&path);
    if (fs == NULL) {
        return FR_NO_PATH;
    }
#endif	
    return dfs_file_unlink(path);
}

FRESULT f_rename_helper(const TCHAR *path_old, const TCHAR *path_new) {
#if FF_USE_MPY_DRIVER		
    FATFS *fs_old = lookup_path(&path_old);
    if (fs_old == NULL) {
        return FR_NO_PATH;
    }
    FATFS *fs_new = lookup_path(&path_new);
    if (fs_new == NULL) {
        return FR_NO_PATH;
    }
    if (fs_old != fs_new) {
        return FR_NO_PATH;
    }
#endif
	
    //return dfs_file_rename(path_old, path_new);
	return FR_NOT_READY;
}
// When a sector boundary is encountered while writing a file and there are
// more than 512 bytes left to write FatFs will detect that it can bypass
// its internal write buffer and pass the data buffer passed to it directly
// to the disk write function. However, the disk write function needs the
// buffer to be aligned to a 4-byte boundary. FatFs doesn't know this and
// will pass an unaligned buffer if we don't fix the issue. To fix this problem
// we use a temporary buffer to fix the alignment and to speed everything up.

// We use this temporary buffer for both reads and writes. The buffer allows us
// to do multi-block reads and writes which signifcantly speed things up.

static uint32_t file_buffer_offset = 0;
static uint8_t *file_buffer_pointer = 0;
static uint32_t file_buffer_size = 0;
static uint32_t file_buffer_index = 0;

void file_buffer_init0()
{
    file_buffer_offset = 0;
    file_buffer_pointer = 0;
    file_buffer_size = 0;
    file_buffer_index = 0;
}

ALWAYS_INLINE static void file_fill(FIL *fp)
{
    if (file_buffer_index == file_buffer_size) {
        file_buffer_pointer -= file_buffer_offset;
        file_buffer_size += file_buffer_offset;
        file_buffer_offset = 0;
        file_buffer_index = 0;
        uint32_t file_remaining = f_size(fp) - f_tell(fp);
        uint32_t can_do = FF_MIN(file_buffer_size, file_remaining);
        UINT bytes;
        bytes = dfs_file_read(fp, file_buffer_pointer, can_do);
        if (bytes != can_do) ff_read_fail(fp);
    }
}

ALWAYS_INLINE static void file_flush(FIL *fp)
{
    if (file_buffer_index == file_buffer_size) {
        UINT bytes;
        bytes = dfs_file_write(fp, file_buffer_pointer, file_buffer_index);
        if (bytes != file_buffer_index) ff_write_fail(fp);
        file_buffer_pointer -= file_buffer_offset;
        file_buffer_size += file_buffer_offset;
        file_buffer_offset = 0;
        file_buffer_index = 0;
    }
}

uint32_t file_tell_w_buf(FIL *fp)
{
    if ((fp->flags & O_RDONLY) == O_RDONLY) {
        return fp->pos - file_buffer_size + file_buffer_index;
    } else {
        return fp->pos + file_buffer_index;
    }
}

uint32_t file_size_w_buf(FIL *fp)
{
    if ((fp->flags & O_RDONLY) == O_RDONLY) {
        return fp->size;
    } else {
        return fp->size + file_buffer_index;
    }
}

void file_buffer_on(FIL *fp)
{
    file_buffer_offset = f_tell(fp) % 4;
    file_buffer_pointer = fb_alloc_all(&file_buffer_size, FB_ALLOC_PREFER_SIZE) + file_buffer_offset;
    if (!file_buffer_size) {
        nlr_raise(mp_obj_new_exception_msg(&mp_type_MemoryError, "No memory!"));
    }
    file_buffer_size -= file_buffer_offset;
    file_buffer_index = 0;
    if ((fp->flags & O_RDONLY) == O_RDONLY) {
        uint32_t file_remaining = f_size(fp) - f_tell(fp);
        uint32_t can_do = FF_MIN(file_buffer_size, file_remaining);
        UINT bytes;
        bytes = dfs_file_read(fp, file_buffer_pointer, can_do);
        if (bytes != can_do) ff_read_fail(fp);
    }
}

void file_buffer_off(FIL *fp)
{
    if ((fp->flags & O_RDWR) && file_buffer_index) {
        UINT bytes;
        bytes = dfs_file_write(fp, file_buffer_pointer, file_buffer_index);
        if (bytes != file_buffer_index) ff_write_fail(fp);
    }
    file_buffer_pointer = 0;
    fb_free();
}

void read_byte(FIL *fp, uint8_t *value)
{
    if (file_buffer_pointer) {
        // We get a massive speed boost by buffering up as much data as possible
        // via massive reads. So much so that the time wasted by
        // all these operations does not cost us.
        for (size_t i = 0; i < sizeof(*value); i++) {
            file_fill(fp);
            ((uint8_t *) value)[i] = file_buffer_pointer[file_buffer_index++];
        }
    } else {
        UINT bytes;
        bytes = dfs_file_read(fp, value, sizeof(*value));
        if (bytes != sizeof(*value)) ff_read_fail(fp);
    }
}

void read_byte_expect(FIL *fp, uint8_t value)
{
    uint8_t compare;
    read_byte(fp, &compare);
    if (value != compare) ff_expect_fail(fp);
}

void read_byte_ignore(FIL *fp)
{
    uint8_t trash;
    read_byte(fp, &trash);
}

void read_word(FIL *fp, uint16_t *value)
{
    if (file_buffer_pointer) {
        // We get a massive speed boost by buffering up as much data as possible
        // via massive reads. So much so that the time wasted by
        // all these operations does not cost us.
        for (size_t i = 0; i < sizeof(*value); i++) {
            file_fill(fp);
            ((uint8_t *) value)[i] = file_buffer_pointer[file_buffer_index++];
        }
    } else {
        UINT bytes;
        bytes = dfs_file_read(fp, value, sizeof(*value));
        if (bytes != sizeof(*value)) ff_read_fail(fp);
    }
}

void read_word_expect(FIL *fp, uint16_t value)
{
    uint16_t compare;
    read_word(fp, &compare);
    if (value != compare) ff_expect_fail(fp);
}

void read_word_ignore(FIL *fp)
{
    uint16_t trash;
    read_word(fp, &trash);
}

void read_long(FIL *fp, uint32_t *value)
{
    if (file_buffer_pointer) {
        // We get a massive speed boost by buffering up as much data as possible
        // via massive reads. So much so that the time wasted by
        // all these operations does not cost us.
        for (size_t i = 0; i < sizeof(*value); i++) {
            file_fill(fp);
            ((uint8_t *) value)[i] = file_buffer_pointer[file_buffer_index++];
        }
    } else {
        UINT bytes;
        bytes = dfs_file_read(fp, value, sizeof(*value));
        if (bytes != sizeof(*value)) ff_read_fail(fp);
    }
}

void read_long_expect(FIL *fp, uint32_t value)
{
    uint32_t compare;
    read_long(fp, &compare);
    if (value != compare) ff_expect_fail(fp);
}

void read_long_ignore(FIL *fp)
{
    uint32_t trash;
    read_long(fp, &trash);
}

void read_data(FIL *fp, void *data, UINT size)
{
    if (file_buffer_pointer) {
        // We get a massive speed boost by buffering up as much data as possible
        // via massive reads. So much so that the time wasted by
        // all these operations does not cost us.
        while (size) {
            file_fill(fp);
            uint32_t file_buffer_space_left = file_buffer_size - file_buffer_index;
            uint32_t can_do = FF_MIN(size, file_buffer_space_left);
            memcpy(data, file_buffer_pointer+file_buffer_index, can_do);
            file_buffer_index += can_do;
            data += can_do;
            size -= can_do;
        }
    } else {
        UINT bytes;
        bytes = dfs_file_read(fp, data, size);
        if (bytes != size) ff_read_fail(fp);
    }
}

void write_byte(FIL *fp, uint8_t value)
{
    if (file_buffer_pointer) {
        // We get a massive speed boost by buffering up as much data as possible
        // before a write to the SD card. So much so that the time wasted by
        // all these operations does not cost us.
        for (size_t i = 0; i < sizeof(value); i++) {
            file_buffer_pointer[file_buffer_index++] = ((uint8_t *) &value)[i];
            file_flush(fp);
        }
    } else {
        UINT bytes;
        bytes = dfs_file_write(fp, &value, sizeof(value));
        if (bytes != sizeof(value)) ff_write_fail(fp);
    }
}

void write_word(FIL *fp, uint16_t value)
{
    if (file_buffer_pointer) {
        // We get a massive speed boost by buffering up as much data as possible
        // before a write to the SD card. So much so that the time wasted by
        // all these operations does not cost us.
        for (size_t i = 0; i < sizeof(value); i++) {
            file_buffer_pointer[file_buffer_index++] = ((uint8_t *) &value)[i];
            file_flush(fp);
        }
    } else {
        UINT bytes;
        bytes = dfs_file_write(fp, &value, sizeof(value));
        if (bytes != sizeof(value)) ff_write_fail(fp);
    }
}

void write_long(FIL *fp, uint32_t value)
{
    if (file_buffer_pointer) {
        // We get a massive speed boost by buffering up as much data as possible
        // before a write to the SD card. So much so that the time wasted by
        // all these operations does not cost us.
        for (size_t i = 0; i < sizeof(value); i++) {
            file_buffer_pointer[file_buffer_index++] = ((uint8_t *) &value)[i];
            file_flush(fp);
        }
    } else {
        UINT bytes;
        bytes = dfs_file_write(fp, &value, sizeof(value));
        if (bytes != sizeof(value)) ff_write_fail(fp);
    }
}

void write_data(FIL *fp, const void *data, UINT size)
{
    if (file_buffer_pointer) {
        // We get a massive speed boost by buffering up as much data as possible
        // before a write to the SD card. So much so that the time wasted by
        // all these operations does not cost us.
        while (size) {
            uint32_t file_buffer_space_left = file_buffer_size - file_buffer_index;
            uint32_t can_do = FF_MIN(size, file_buffer_space_left);
            memcpy(file_buffer_pointer+file_buffer_index, data, can_do);
            file_buffer_index += can_do;
            data += can_do;
            size -= can_do;
            file_flush(fp);
        }
    } else {
        UINT bytes;
        bytes = dfs_file_write(fp, data, size);
        if (bytes != size) ff_write_fail(fp);
    }
}
