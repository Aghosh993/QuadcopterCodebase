/*
 * newlib_stubs.c
 *
 *  Created on: 2 Nov 2010
 *      Author: nanoage.co.uk
 */
#include <errno.h>
#include <sys/stat.h>
#include <sys/times.h>
#include <sys/unistd.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>

#ifndef STDIN_USART
#define STDIN_USART     2
#endif

#ifndef STDOUT_USART
#define STDOUT_USART    2
#endif

#ifndef STDERR_USART
#define STDERR_USART    2
#endif

#undef errno
extern int errno;

#define __ASM            __asm                                      /*!< asm keyword for GNU Compiler          */
#define __INLINE         inline                                     /*!< inline keyword for GNU Compiler       */
#define __STATIC_INLINE  static inline

/** \brief  Get Main Stack Pointer

    This function returns the current value of the Main Stack Pointer (MSP).

    \return               MSP Register value
 */
__attribute__( ( always_inline ) ) __STATIC_INLINE uint32_t __get_MSP(void)
{
  register uint32_t result;

  __ASM volatile ("MRS %0, msp\n" : "=r" (result) );
  return(result);
}

/*
 environ
 A pointer to a list of environment variables and their values.
 For a minimal environment, this empty list is adequate:
 */
char *__env[1] = { 0 };
char **environ = __env;

int _write(int file, char *ptr, int len);

void _exit(int status) {
    _write(1, "exit", 4);
    while (1) {
        ;
    }
}

int _close(int file) {
    return -1;
}
/*
 execve
 Transfer control to a new process. Minimal implementation (for a system without processes):
 */
int _execve(char *name, char **argv, char **env) {
    errno = ENOMEM;
    return -1;
}
/*
 fork
 Create a new process. Minimal implementation (for a system without processes):
 */

int _fork() {
    errno = EAGAIN;
    return -1;
}
/*
 fstat
 Status of an open file. For consistency with other minimal implementations in these examples,
 all files are regarded as character special devices.
 The `sys/stat.h' header file required is distributed in the `include' subdirectory for this C library.
 */
int _fstat(int file, struct stat *st) {
    st->st_mode = S_IFCHR;
    return 0;
}

/*
 getpid
 Process-ID; this is sometimes used to generate strings unlikely to conflict with other processes. Minimal implementation, for a system without processes:
 */

int _getpid() {
    return 1;
}

/*
 isatty
 Query whether output stream is a terminal. For consistency with the other minimal implementations,
 */
int _isatty(int file) {
    switch (file){
    case STDOUT_FILENO:
    case STDERR_FILENO:
    case STDIN_FILENO:
        return 1;
    default:
        //errno = ENOTTY;
        errno = EBADF;
        return 0;
    }
}


/*
 kill
 Send a signal. Minimal implementation:
 */
int _kill(int pid, int sig) {
    errno = EINVAL;
    return (-1);
}

/*
 link
 Establish a new name for an existing file. Minimal implementation:
 */

int _link(char *old, char *new) {
    errno = EMLINK;
    return -1;
}

/*
 lseek
 Set position in a file. Minimal implementation:
 */
int _lseek(int file, int ptr, int dir) {
    return 0;
}

/*
 sbrk
 Increase program data space.
 Malloc and related functions depend on this
 */
caddr_t _sbrk(int incr) {

    extern char _ebss; // Defined by the linker
    static char *heap_end;
    char *prev_heap_end;

    if (heap_end == 0) {
        heap_end = &_ebss;
    }
    prev_heap_end = heap_end;

char * stack = (char*) __get_MSP();
     if (heap_end + incr >  stack)
     {
         _write (STDERR_FILENO, "Heap and stack collision\n", 25);
         errno = ENOMEM;
         return  (caddr_t) -1;
         //abort ();
     }

    heap_end += incr;
    return (caddr_t) prev_heap_end;

}

// #define HEAPSIZE 32768

// unsigned char _heap[HEAPSIZE];

// caddr_t _sbrk(int incr);
// caddr_t _sbrk(int incr) {
//   write(STDERR_USART, "a\n", 2);
//   static unsigned char *heap_end;
//   unsigned char *prev_heap_end;
//   ///* debugging
//   ///*
//   static int first=0;
//   if (first==2) {
//     write( 2, "Asking for: ", 12);
//     char incr_c[15];
//     // itoa(incr, incr_c);
//     // write( 2, incr_c, strlen2(incr_c));
//     // write( 2, " bytes.\n", 8);
//   } else {
//     first++;
//   }
//   //*/
//   //
// /* initialize */
//   if( heap_end == 0 ) heap_end = _heap;
//   prev_heap_end = heap_end;
//   if( heap_end + incr -_heap > HEAPSIZE ) {
// /* heap overflow—announce on stderr */
//     write( 2, "Heap overflow!\n", 15 );
//     abort();
//   }
//   heap_end += incr;
//   return (caddr_t) prev_heap_end;
// }

/*
 read
 Read a character to a file. `libc' subroutines will use this system routine for input from all files, including stdin
 Returns -1 on error or blocks until the number of characters have been read.
 */


int _read(int file, char *ptr, int len) {
    int n;
    int num = 0;
    char c;
    switch (file) {
    case STDIN_FILENO:
        for (n = 0; n < len; n++) {
#if   STDIN_USART == 1
            c = (char)usart_recv_blocking(USART1);
#elif STDIN_USART == 2
            c = (char)usart_recv_blocking(USART2);
#elif STDIN_USART == 3
            c = (char)usart_recv_blocking(USART3);
#endif
            *ptr++ = c;
            num++;
        }
        break;
    default:
        errno = EBADF;
        return -1;
    }
    return num;
}

/*
 stat
 Status of a file (by name). Minimal implementation:
 int    _EXFUN(stat,( const char *__path, struct stat *__sbuf ));
 */

int _stat(const char *filepath, struct stat *st) {
    st->st_mode = S_IFCHR;
    return 0;
}

/*
 times
 Timing information for current process. Minimal implementation:
 */

clock_t _times(struct tms *buf) {
    return -1;
}

/*
 unlink
 Remove a file's directory entry. Minimal implementation:
 */
int _unlink(char *name) {
    errno = ENOENT;
    return -1;
}

/*
 wait
 Wait for a child process. Minimal implementation:
 */
int _wait(int *status) {
    errno = ECHILD;
    return -1;
}

/*
 write
 Write a character to a file. `libc' subroutines will use this system routine for output to all files, including stdout
 Returns -1 on error or number of bytes sent
 */
int _write(int file, char *ptr, int len) {
    int n;
    switch (file) {
    case STDOUT_FILENO: /*stdout*/
        for (n = 0; n < len; n++) {
#if STDOUT_USART == 1
            usart_send_blocking(USART1, (uint16_t)*ptr++);
#elif  STDOUT_USART == 2
            usart_send_blocking(USART2, (uint16_t)*ptr++);
#elif  STDOUT_USART == 3
            usart_send_blocking(USART3, (uint16_t)*ptr++);
#endif
        }
        break;
    case STDERR_FILENO: /* stderr */
        for (n = 0; n < len; n++) {
#if STDERR_USART == 1
            usart_send_blocking(USART1, (uint16_t)*ptr++);
#elif  STDERR_USART == 2
            usart_send_blocking(USART2, (uint16_t)*ptr++);
#elif  STDERR_USART == 3
            usart_send_blocking(USART3, (uint16_t)*ptr++);
#endif
        }
        break;
    default:
        errno = EBADF;
        return -1;
    }
    return len;
}
