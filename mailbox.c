/*
Copyright (c) 2012, Broadcom Europe Ltd.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <assert.h>
#include <stdint.h>
#include <errno.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/sysmacros.h>

#include "mailbox.h"
//#define DEBUG

#define PAGE_SIZE (4*1024)

void *mapmem(unsigned base, unsigned size, const char *mem_dev) {
   unsigned offset = base % PAGE_SIZE;
   base = base - offset;

   /* open /dev/mem or /dev/gpiomem */
   int mem_fd = open(mem_dev, O_RDWR | O_SYNC);
   if (mem_fd < 0) {
      fprintf(stderr, "can't open %s\nThis program should be run as root. Try prefixing command with: sudo\n", mem_dev);
      return NULL;
   }
   void *mem = mmap(
      0,
      size,
      PROT_READ|PROT_WRITE,
      MAP_SHARED/*|MAP_FIXED*/,
      mem_fd,
      base);
#ifdef DEBUG
   printf("base=0x%x, mem=%p\n", base, mem);
#endif
   if (mem == MAP_FAILED) {
      fprintf(stderr, "mmap error %d\n", (int) mem);
      return NULL;
   }
   close(mem_fd);
   return (char *)mem + offset;
}

void *unmapmem(void *addr, unsigned size) {
   uint32_t pagemask = ~0UL ^ (getpagesize() - 1);
   uintptr_t baseaddr = (uintptr_t)addr & pagemask;

   int s = munmap((void *)baseaddr, size);
   if (s != 0) {
      fprintf(stderr, "munmap error %d\n", s);
   }

   return NULL;
}

/*
 * use ioctl to send mbox property message
 */

static int mbox_property(int file_desc, void *buf) {
   int fd = file_desc;
   int ret_val = -1;

   if (fd < 0) {
      fd = mbox_open();
   }

   if (fd >= 0) {
      ret_val = ioctl(fd, IOCTL_MBOX_PROPERTY, buf);

      if (ret_val < 0) {
         fprintf(stderr, "ioctl_set_msg failed, errno %d: %m\n", errno);
      }
   }
#ifdef DEBUG
   unsigned *p = buf; int i; unsigned size = *(unsigned *)buf;
   for (i=0; i<size/4; i++)
      printf("%04x: 0x%08x\n", i*sizeof *p, p[i]);
#endif
   if (file_desc < 0)
      mbox_close(fd);

   return ret_val;
}

unsigned mem_alloc(int file_desc, unsigned size, unsigned align, unsigned flags) {
   int i=0;
   unsigned p[32];

#ifdef DEBUG
   printf("Requesting %d bytes\n", size);
#endif
   p[i++] = 0; // size
   p[i++] = 0x00000000; // process request

   p[i++] = 0x3000c; // (the tag id)
   p[i++] = 12; // (size of the buffer)
   p[i++] = 12; // (size of the data)
   p[i++] = size; // (num bytes? or pages?)
   p[i++] = align; // (alignment)
   p[i++] = flags; // (MEM_FLAG_L1_NONALLOCATING)

   p[i++] = 0x00000000; // end tag
   p[0] = i*sizeof *p; // actual size

   if (mbox_property(file_desc, p) < 0)
      return -1;
   else
      return p[5];
}

unsigned mem_free(int file_desc, unsigned handle) {
   int i=0;
   unsigned p[32];

   p[i++] = 0; // size
   p[i++] = 0x00000000; // process request

   p[i++] = 0x3000f; // (the tag id)
   p[i++] = 4; // (size of the buffer)
   p[i++] = 4; // (size of the data)
   p[i++] = handle;

   p[i++] = 0x00000000; // end tag
   p[0] = i * sizeof *p; // actual size

   mbox_property(file_desc, p);

   return p[5];
}

unsigned mem_lock(int file_desc, unsigned handle) {
   int i = 0;
   unsigned p[32];

   p[i++] = 0; // size
   p[i++] = 0x00000000; // process request

   p[i++] = 0x3000d; // (the tag id)
   p[i++] = 4; // (size of the buffer)
   p[i++] = 4; // (size of the data)
   p[i++] = handle;

   p[i++] = 0x00000000; // end tag
   p[0] = i*sizeof *p; // actual size

   if (mbox_property(file_desc, p) < 0)
      return ~0;
   else
      return p[5];
}

unsigned mem_unlock(int file_desc, unsigned handle) {
   int i = 0;
   unsigned p[32];
   p[i++] = 0; // size
   p[i++] = 0x00000000; // process request

   p[i++] = 0x3000e; // (the tag id)
   p[i++] = 4; // (size of the buffer)
   p[i++] = 4; // (size of the data)
   p[i++] = handle;

   p[i++] = 0x00000000; // end tag
   p[0] = i*sizeof *p; // actual size

   mbox_property(file_desc, p);
   return p[5];
}

unsigned execute_code(int file_desc, unsigned code, unsigned r0, unsigned r1, unsigned r2, unsigned r3, unsigned r4, unsigned r5) {
   int i=0;
   unsigned p[32];

   p[i++] = 0; // size
   p[i++] = 0x00000000; // process request

   p[i++] = 0x30010; // (the tag id)
   p[i++] = 28; // (size of the buffer)
   p[i++] = 28; // (size of the data)
   p[i++] = code;
   p[i++] = r0;
   p[i++] = r1;
   p[i++] = r2;
   p[i++] = r3;
   p[i++] = r4;
   p[i++] = r5;

   p[i++] = 0x00000000; // end tag
   p[0] = i*sizeof *p; // actual size

   mbox_property(file_desc, p);
   return p[5];
}

unsigned qpu_enable(int file_desc, unsigned enable) {
   int i=0;
   unsigned p[32];

   p[i++] = 0; // size
   p[i++] = 0x00000000; // process request

   p[i++] = 0x30012; // (the tag id)
   p[i++] = 4; // (size of the buffer)
   p[i++] = 4; // (size of the data)
   p[i++] = enable;

   p[i++] = 0x00000000; // end tag
   p[0] = i * sizeof *p; // actual size

   mbox_property(file_desc, p);

   return p[5];
}

unsigned execute_qpu(int file_desc, unsigned num_qpus, unsigned control, unsigned noflush, unsigned timeout) {
   int i=0;
   unsigned p[32];

   p[i++] = 0; // size
   p[i++] = 0x00000000; // process request
   p[i++] = 0x30011; // (the tag id)
   p[i++] = 16; // (size of the buffer)
   p[i++] = 16; // (size of the data)
   p[i++] = num_qpus;
   p[i++] = control;
   p[i++] = noflush;
   p[i++] = timeout; // ms

   p[i++] = 0x00000000; // end tag
   p[0] = i*sizeof *p; // actual size

   mbox_property(file_desc, p);
   return p[5];
}

int mbox_open(void) {
   int file_desc;
   char filename[64];

   // open a char device file used for communicating with kernel mbox driver
   if ((file_desc = open("/dev/vcio", 0)) >= 0) {
      /* New kernel, we use /dev/vcio, major 249, since kernel 4.1 */
      return file_desc;
   }

   /* Most likely an old kernel, so drop back to the old major=100 device */
   sprintf(filename, "/tmp/mailbox-%d", getpid());
   unlink(filename);
   if (mknod(filename, S_IFCHR|0600, makedev(100, 0)) < 0) {
      fprintf(stderr, "Failed to create mailbox device %s: %m\n", filename);
      return -1;
   }
   file_desc = open(filename, 0);
   if (file_desc < 0) {
      fprintf(stderr, "Can't open device file %s: %m\n", filename);
      unlink(filename);
      return -1;
   }
   unlink(filename);

   return file_desc;
}

void mbox_close(int file_desc) {
   close(file_desc);
}
