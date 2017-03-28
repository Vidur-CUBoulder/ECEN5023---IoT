/*
 * circular_buffer.c
 *
 *  Created on: Mar 26, 2017
 *      Author: vidursarin
 */

#include "circular_buffer.h"


debug_buf Alloc_Buffer(c_buf *buffer, uint8_t size)
{
  buffer->buf_start = (c_buf *)malloc(sizeof(uint8_t) * size);
  buffer->buf_end = buffer->buf_start + (sizeof(uint8_t) * (size-1));
 
  /* By default, set all the elements of the array to the null char. */
  memset(buffer->buf_start, '\0', (sizeof(uint8_t)*size));

  buffer->size_of_buffer = size;
  buffer->elements = 0;
  buffer->underflow = true;
  buffer->overflow = false;
  buffer->tail = buffer->head = buffer->buf_start;

  return SUCCESS;
}

debug_buf add_to_buffer(c_buf *buffer, void *data, uint8_t multiplier)
{
  if(buffer == NULL) {
    return NULL_RETURN;
  }

  if(buffer->overflow == true) {
    /* Since the overflow flag is set, we need to do a 
     * check every time that we try and add data to the 
     * buffer.
     */
    uint8_t chk_data;
    memcpy(&chk_data, buffer->head, multiplier);
    
    if(chk_data != '\0') {
    /* There isn't any space, exit! */
#ifdef DEBUG_CIRC_BUF
      printf("Position not empty, exit!\n");
#endif
      return BUFFER_FULL;
    } else {
      /* Valid: We have space, enter the data, just pass through */
#ifdef DEBUG_CIRC_BUF
      printf("Enter the data!\n");
#endif
    }
  }

  buffer->underflow = (buffer->elements == 0) ? true:false;
  memcpy(buffer->head, data, multiplier);
  buffer->head = buffer->head + multiplier;

  /* Check if the bounds of the buffer have been surpassed.
   * If yes, move the head pointer to the next element in the 
   * memory & set the overflow flag
   */
  if(buffer->head > buffer->buf_end) {
    buffer->head = buffer->buf_start;
    buffer->overflow = true;
  }
  buffer->elements = buffer->elements + multiplier;

  return SUCCESS;
}

debug_buf remove_from_buffer(c_buf *buffer, void *ret_data, uint8_t multiplier)
{
  if(buffer == NULL) {
    return NULL_RETURN;
  }

  buffer->underflow = (buffer->elements == 0) ? true:false;
  if(buffer->underflow == true) {
  /* The underflow flag is set, there is nothing to remove, EXIT! */
#ifdef DEBUG_CIRC_BUF
    printf("Nothing to remove from an empty buffer\n");
#endif
    return BUFFER_EMPTY;
  }

  uint8_t null_data = '\0';

  /* Copy a null char to the empty data space */
  memcpy(ret_data, buffer->tail, multiplier);
  memset(buffer->tail, '\0', multiplier);
  buffer->tail = buffer->tail + multiplier;
  buffer->elements = buffer->elements - multiplier;

  return SUCCESS;
}

#ifdef DEBUG_CIRC_BUF
void Debug_Buffer(c_buf *buffer, uint8_t size)
{

  void *temp_buffer = buffer->buf_start;
  uint8_t data = 0;
  uint8_t count = 0;
  do{
    printf("temp_buffer: %p\n", temp_buffer); 
    memcpy(&data, temp_buffer, sizeof(uint8_t));
    printf("data: %x\n", data);

    temp_buffer = temp_buffer + sizeof(uint8_t);
    count++;
  } while(count != size);

  return;
}
#endif

debug_buf free_buffer(void *buffer)
{
  free(buffer);

  return BUFFER_FREE;
}

#if 0
int main()
{
  c_buf *temp_buf;
  buffer = temp_buf;

  float data_f = 12.54;
  uint8_t data_8 = 3;

  Alloc_Buffer(buffer, 5);
  printf("buffer->start: %p\n", buffer->buf_start);
  printf("buffer->end: %p\n", buffer->buf_end);
  
  add_to_buffer(buffer, &data_f, sizeof(float));
  add_to_buffer(buffer, &data_8, sizeof(uint8_t));
  remove_from_buffer(buffer, sizeof(float)); 
  add_to_buffer(buffer, &data_8, sizeof(uint8_t));
  
  Debug_Buffer(buffer, 5);

  free_buffer(buffer->buf_start);
  
  return 0;
}
#endif


