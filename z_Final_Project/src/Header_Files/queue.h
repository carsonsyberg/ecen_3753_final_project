#include <stdio.h>
#include <stdlib.h>

#ifndef __QUEUE__
#define __QUEUE__

enum Button_State {PRESSED = 1, UNPRESSED = 2};

//----------------------------------------------------------------------------------------------------------------------------------
/// @brief Structure which holds the Queue's Node information
//----------------------------------------------------------------------------------------------------------------------------------
struct node_t {
    // state of the respective button
    int button_state;

    // Pointer to the next node in the queue
    struct node_t* next;
};

//----------------------------------------------------------------------------------------------------------------------------------
/// @brief Creates a queue with single node and button_state = 0.
///
/// @return the head of the new queue
//----------------------------------------------------------------------------------------------------------------------------------
struct node_t* create_queue();

//----------------------------------------------------------------------------------------------------------------------------------
/// @brief Create a new node for the queue
///
/// @param btn_state The state of the button after a change in state of the GPIO.
///
/// @return a newly allocated node in the FIFO
//----------------------------------------------------------------------------------------------------------------------------------
struct node_t* create_new_node(int btn_state);

//----------------------------------------------------------------------------------------------------------------------------------
/// @brief Returns the top node in the queue
///
/// @param head The head of the queue
///
/// @return the btn_state at the top of the queue
//----------------------------------------------------------------------------------------------------------------------------------
int peek(struct node_t** head);

//----------------------------------------------------------------------------------------------------------------------------------
/// @brief Removes the element at the top of the queue.
///
/// @param head The head of the queue.
///
/// @return the btn_state being popped from the queue
//----------------------------------------------------------------------------------------------------------------------------------
int pop(struct node_t** head);

//----------------------------------------------------------------------------------------------------------------------------------
/// @brief Push a new task into the queue
///
/// @param head The head of the queue
/// @param btn_state The state of the button after a change in state of the GPIO.
//----------------------------------------------------------------------------------------------------------------------------------
void push(struct node_t** head, int btn_state);

//----------------------------------------------------------------------------------------------------------------------------------
/// @brief Determines whether the specified head is empty.
///
/// @param head The head of the Queue
///
/// @return True if the specified head is empty, False otherwise.
//----------------------------------------------------------------------------------------------------------------------------------
int is_empty(struct node_t** head);

//----------------------------------------------------------------------------------------------------------------------------------
/// @brief Remove all items from the queue
///
/// @param head The head of the queue
//----------------------------------------------------------------------------------------------------------------------------------
void empty_queue(struct node_t** head);

#endif // __QUEUE__
