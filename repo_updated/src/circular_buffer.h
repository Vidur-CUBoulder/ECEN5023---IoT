/*
 * circular_buffer.h
 *
 *  Created on: Mar 26, 2017
 *      Author: vidursarin
 */

#ifndef SRC_CIRCULAR_BUFFER_H_
#define SRC_CIRCULAR_BUFFER_H_

#include<stdio.h>
#include<stdlib.h>
#include<stdint.h>
#include<string.h>
#include<stdbool.h>

//#define DEBUG_CIRC_BUF

typedef struct circular_buffer {
  void *buf_start;
  void *buf_end;
  void *head;
  void *tail;
  uint8_t size_of_buffer;
  uint8_t elements;
  bool underflow;
  bool overflow;
} c_buf;

/* A globally defined buffer */
c_buf buffer;

typedef enum debug_circular_buffer {
  NULL_RETURN = 0,
  SUCCESS = 1,
  BUFFER_FULL = 2,
  BUFFER_EMPTY = 3,
  BUFFER_FREE = 4,
  UKNOWN_FAILURE = 5
} debug_buf;

/* Function: Alloc_Buffer(c_buf *buffer, uint8_t size)
 * Input Parameters:
 *      - c_buf *buffer: pointer to the buffer type to alloc.
 *      - uint8_t size: size of the buffer to allocate
 * Return Values:
 *      - debug_buf: debug handle to debug and interpret the 
 *          function.
 * Description: 
 *      - Use this function to allocate space for the cirular buffer
 *        You have to specify the size of the buffer that you want to allocate
 */
debug_buf Alloc_Buffer(c_buf *buffer, uint8_t size);

/* Function: add_to_buffer(c_buf *buffer, uint8_t data)
 * Input Parameters:
 *      - c_buf *buffer: pointer to the buffer.
 *      - uint8_t data: data that you want to enter in the memory
 *                      location pointed by the head.
 * Return Values:
 *      - debug_buf: debug handle to debug and interpret the func. 
 * Description: 
 *      - Use this function to add a value to a memory location that is 
 *        allocated to the circular buffer.
 */
debug_buf add_to_buffer(c_buf *buffer, void *data, uint8_t multiplier);

/* Function: remove_from_buffer(c_buf *buffer);
 * Input Parameters:
 *      - c_buf *buffer: pointer to the buffer 
 * Return Values: 
 *      - debug_buf: debug handle to debug and interpret the func. 
 * Description:
 *      - Use this function to remove an element from the circular buffer
 */
debug_buf remove_from_buffer(c_buf *buffer, void *ret_data, uint8_t multiplier);

/* Function: Debug_Buffer(c_buf *buffer, uint8_t size);
 * Input Parameters:
 *      - c_buf *buffer: pointer to the buffer
 *      - uint8_t size: size of the buffer that was created
 * Return Values: 
 *      - debug_buf: debug handle to debug and interpret the func. 
 * Description:
 *      - This is only a debug function. Will output the entire circular 
 *        buffer and the elements that are stored at every memory location.
 */
void Debug_Buffer(c_buf *buffer, uint8_t size);

/* Function: free_buffer(void *buffer)
 * Input Parameters:
 *      - void *buffer: a pointer to the start of the circular buffer
 * Return Values:
 *      - debug_buf: debug handle to debug and interpret the func. 
 * Description:
 *      - Use this function to free the memory space allocated by
 *        the circular buffer.
 */
debug_buf free_buffer(void *buffer);

#endif /* SRC_CIRCULAR_BUFFER_H_ */
