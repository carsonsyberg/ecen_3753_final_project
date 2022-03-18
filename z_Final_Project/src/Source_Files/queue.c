#include <stdio.h>
#include <stdlib.h>
#include "../Header_Files/queue.h"

struct node_t* create_queue() {
    // create the first node and fill it with current btn_state which should be 0
    struct node_t* head_node = malloc(sizeof(struct node_t));
    head_node->button_state = 0;
    head_node->next = NULL;

    // return the head of the new queue (node_t*)
    return head_node;
}

struct node_t* create_new_node(int btn_state) {
    // create a node and fill it with task info
    struct node_t* new_node = malloc(sizeof(struct node_t));
    // printf("Allocated %ld bytes of memory for node %d\n", sizeof(struct node_t), task->process_id);
    new_node->button_state = btn_state;
    new_node->next = NULL;

    // return the newly created node (node_t*)
    return new_node;
}

int peek(struct node_t** head) {
    // return the top node in the queue (node_t*)

    return (*head)->button_state;
}

int pop(struct node_t** head) {
    // remove top element and update the head
        // need a free() in here
    
    // get a holder pointer for head node
    struct node_t* holder = *head;

    // update head to next
    *head = holder->next;

    // get the value off the node we're popping
    int popped_val = holder->button_state;

    // free previous head node
    // printf("Freed memory for node %d\n", holder->task->process_id);
    free(holder);

    // return button_state from popped node
    return popped_val;
}

void push(struct node_t** head, int btn_state) {
    // put a new task into the queue (at the back of the line)
    struct node_t* new_node = create_new_node(btn_state);

    if(*head == NULL) {
        // if the head is NULL, means queue has been emptied and this push needs to be the new head
        new_node->next = NULL;
        *head = new_node;
        return;
    }

    // need to find end of the queue, loop from head til next is null
    struct node_t* current_node = *head;
    // printf("\nHead (%d)\n", current_node->task->process_id);
    while(current_node->next != NULL) {
        current_node = current_node->next;
    } // after this loop finishes, current_node should = the last node in the queue

    // add new node to end of the list (already populated with info and next = NULL)
    current_node->next = new_node;
    // printf("\nAdded node (%d)\n", current_node->next->task->process_id);
    // printf("\nto node (%d)\n", current_node->task->process_id);
    // printf("\nHead: %d\n", (*head)->task->process_id);
}

int is_empty(struct node_t** head) {
    // return 1 if queue empty, return 0 if not
    // queue empty if head not pointing to anything
    if (*head == NULL)
        return 1;
    else
        return 0;
}

void empty_queue(struct node_t** head) {
    // empty queue by deallocating all memory
    // then set head pointer to NULL
        // need a free() in here for every node
    
    // loop through whole queue starting at headnode_t* current_node = *head;
    struct node_t* holder;
    while(*head != NULL) {
        // step 1: get holder pointer to head node
        holder = *head;
        // set head node to next node
        *head = holder->next;

        // free previous head node
        // printf("Freed memory for node %d\n", holder->task->process_id);
        free(holder);

    } 
        
    // repeat until node is NULL
}
