#include "util.h"
#include "stdio.h"

#define DEBUG 1

void * my_malloc(size_t size)
{
	if (DEBUG) printf("my_malloc: requested %d bytes\n\r", size);
	void * ptr = malloc(size);
	if (ptr == NULL) {
		if (DEBUG) printf("my_malloc: error: malloc returned NULL\n\r");
		return NULL;
	}
	int max_addr = ((int) ptr) + size;
	if (max_addr >= STM32F429_MAX_RAM) {
		if (DEBUG) printf("my_malloc: error: range %x:%x outside of RAM\n\r", (int) ptr, max_addr);
		free(ptr);
		return NULL;
	} else {
		if (DEBUG) printf("my_malloc: successfully allocated range %x:%x\n\r", (int) ptr, max_addr);
		return ptr;
	}
}
