// list_test.c
// Standalone laptop build of the DD-task list functions and tests.
// No FreeRTOS dependencies — compile and run with:
//   gcc -Wall -o list_test list_test.c && ./list_test

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

/*-----------------------------------------------------------*/
// Stripped-down type aliases — replace FreeRTOS types with
// standard C equivalents so the structs match the embedded file exactly.
/*-----------------------------------------------------------*/

typedef void* TaskHandle_t;   // opaque handle — NULL is fine for list tests

// pdMS_TO_TICKS on the hardware uses a 1000 Hz tick, so 1 tick == 1 ms.
// On the laptop we just use raw uint32_t millisecond values directly.
#define pdMS_TO_TICKS(ms) ((uint32_t)(ms))

/*-----------------------------------------------------------*/
// Structs — identical to main.c
/*-----------------------------------------------------------*/

typedef enum { PERIODIC, APERIODIC } task_type;

typedef struct {
    TaskHandle_t t_handle;
    task_type    type;
    uint32_t     task_id;
    uint32_t     release_time;
    uint32_t     absolute_deadline;
    uint32_t     completion_time;
} dd_task;

typedef struct dd_task_list {
    dd_task             task;
    struct dd_task_list *next_task;
} dd_task_list;

/*-----------------------------------------------------------*/
// Forward declarations
/*-----------------------------------------------------------*/

dd_task_list* add_task_to_list(dd_task_list **head, dd_task new_task);
dd_task_list* remove_task_from_list(dd_task_list **head, uint32_t task_id);
dd_task_list* merge_sort_task_list(dd_task_list *head);

/*-----------------------------------------------------------*/
// List functions — identical logic to main.c,
// pvPortMalloc -> malloc, vPortFree -> free
/*-----------------------------------------------------------*/

dd_task_list* add_task_to_list(dd_task_list **head, dd_task new_task)
{
    dd_task_list *node = (dd_task_list*)malloc(sizeof(dd_task_list));
    if (node == NULL) return *head;

    node->task      = new_task;
    node->next_task = NULL;

    if (*head == NULL)
    {
        *head = node;
        return *head;
    }

    dd_task_list *curr = *head;
    while (curr->next_task != NULL)
        curr = curr->next_task;

    curr->next_task = node;
    return *head;
}

/*-----------------------------------------------------------*/

dd_task_list* remove_task_from_list(dd_task_list **head, uint32_t task_id)
{
    if (*head == NULL) return NULL;

    dd_task_list *curr = *head;
    dd_task_list *prev = NULL;

    while (curr != NULL)
    {
        if (curr->task.task_id == task_id)
        {
            if (prev == NULL)
                *head = curr->next_task;
            else
                prev->next_task = curr->next_task;

            free(curr);
            return *head;
        }
        prev = curr;
        curr = curr->next_task;
    }

    return *head;
}

/*-----------------------------------------------------------*/

dd_task_list* merge_sort_task_list(dd_task_list *head)
{
    if (head == NULL || head->next_task == NULL)
        return head;

    int length = 0;
    dd_task_list *curr = head;
    while (curr != NULL) { length++; curr = curr->next_task; }

    dd_task_list dummy;
    dummy.next_task = head;

    for (int size = 1; size < length; size *= 2)
    {
        dd_task_list *tail = &dummy;
        curr = dummy.next_task;

        while (curr != NULL)
        {
            dd_task_list *left  = curr;
            dd_task_list *right = curr;
            int left_len = 0;

            while (left_len < size && right != NULL)
            {
                right = right->next_task;
                left_len++;
            }

            int right_len = size;

            while (left_len > 0 && right_len > 0 && right != NULL)
            {
                if (left->task.absolute_deadline <= right->task.absolute_deadline)
                {
                    tail->next_task = left;
                    left = left->next_task;
                    left_len--;
                }
                else
                {
                    tail->next_task = right;
                    right = right->next_task;
                    right_len--;
                }
                tail = tail->next_task;
            }

            while (left_len > 0 && left != right)
            {
                tail->next_task = left;
                left = left->next_task;
                left_len--;
                tail = tail->next_task;
            }
            while (right_len > 0 && right != NULL)
            {
                tail->next_task = right;
                right = right->next_task;
                right_len--;
                tail = tail->next_task;
            }

            tail->next_task = NULL;
            curr = right;
        }
    }

    return dummy.next_task;
}

/*-----------------------------------------------------------*/
// Helper — print every node in a list on one line
/*-----------------------------------------------------------*/

static void print_list(const char *label, dd_task_list *head)
{
    printf("  %-12s: ", label);
    if (head == NULL) {
        printf("(empty)");
    } else {
        dd_task_list *curr = head;
        while (curr != NULL) {
            printf("ID=%lu(dl=%lu)", (unsigned long)curr->task.task_id,
                   (unsigned long)curr->task.absolute_deadline);
            if (curr->next_task) printf(" -> ");
            curr = curr->next_task;
        }
    }
    printf("\n");
}

/*-----------------------------------------------------------*/
// Helper — free every node in a list (cleanup between tests)
/*-----------------------------------------------------------*/

static void free_list(dd_task_list **head)
{
    dd_task_list *curr = *head;
    while (curr != NULL)
    {
        dd_task_list *next = curr->next_task;
        free(curr);
        curr = next;
    }
    *head = NULL;
}

/*-----------------------------------------------------------*/
// Tests — identical scenarios to List_Test_Task in main.c
/*-----------------------------------------------------------*/

int main(void)
{
    dd_task_list *test_list = NULL;
    dd_task_list *curr      = NULL;
    int pass                = 1;

    printf("\n========================================\n");
    printf("  DD-Task List Function Tests\n");
    printf("  Based on Test Bench #1 (T1=500ms, T2=500ms, T3=750ms)\n");
    printf("========================================\n\n");

    // Three tasks matching Test Bench #1 first hyper-period
    dd_task task1 = {
        .t_handle          = NULL,
        .type              = PERIODIC,
        .task_id           = 1,
        .release_time      = 0,
        .absolute_deadline = pdMS_TO_TICKS(500),
        .completion_time   = 0
    };

    dd_task task2 = {
        .t_handle          = NULL,
        .type              = PERIODIC,
        .task_id           = 2,
        .release_time      = 0,
        .absolute_deadline = pdMS_TO_TICKS(500),
        .completion_time   = 0
    };

    dd_task task3 = {
        .t_handle          = NULL,
        .type              = PERIODIC,
        .task_id           = 3,
        .release_time      = 0,
        .absolute_deadline = pdMS_TO_TICKS(750),
        .completion_time   = 0
    };

    // ------------------------------------------------------------------
    // TEST 1: add_task_to_list — add all three, verify count and order
    // ------------------------------------------------------------------
    printf("--- TEST 1: add_task_to_list ---\n");

    add_task_to_list(&test_list, task3);  // intentionally out of deadline order
    add_task_to_list(&test_list, task1);
    add_task_to_list(&test_list, task2);

    int count = 0;
    curr = test_list;
    while (curr != NULL) { count++; curr = curr->next_task; }

    if (count == 3)
        printf("  [PASS] List has 3 nodes after 3 adds\n");
    else {
        printf("  [FAIL] Expected 3 nodes, got %d\n", count);
        pass = 0;
    }

    if (test_list != NULL &&
        test_list->task.task_id == 3 &&
        test_list->next_task->task.task_id == 1 &&
        test_list->next_task->next_task->task.task_id == 2)
        printf("  [PASS] Insertion order preserved (3 -> 1 -> 2)\n");
    else {
        printf("  [FAIL] Insertion order wrong\n");
        pass = 0;
    }

    print_list("Before sort", test_list);
    printf("\n");

    // ------------------------------------------------------------------
    // TEST 2: merge_sort_task_list — sort by absolute_deadline ascending
    // ------------------------------------------------------------------
    printf("--- TEST 2: merge_sort_task_list ---\n");

    test_list = merge_sort_task_list(test_list);

    int sort_pass = 1;
    curr = test_list;
    if (curr == NULL || curr->task.absolute_deadline != pdMS_TO_TICKS(500)) sort_pass = 0;
    else curr = curr->next_task;
    if (curr == NULL || curr->task.absolute_deadline != pdMS_TO_TICKS(500)) sort_pass = 0;
    else curr = curr->next_task;
    if (curr == NULL || curr->task.absolute_deadline != pdMS_TO_TICKS(750)) sort_pass = 0;

    if (sort_pass)
        printf("  [PASS] Sorted by deadline: 500 -> 500 -> 750\n");
    else {
        printf("  [FAIL] Sort order incorrect\n");
        pass = 0;
    }

    print_list("After sort", test_list);
    printf("\n");

    // ------------------------------------------------------------------
    // TEST 3: remove_task_from_list — remove middle node
    // ------------------------------------------------------------------
    printf("--- TEST 3: remove_task_from_list (middle node) ---\n");

    uint32_t mid_id = test_list->next_task->task.task_id;
    test_list = remove_task_from_list(&test_list, mid_id);

    count = 0;
    int found_removed = 0;
    curr = test_list;
    while (curr != NULL) {
        if (curr->task.task_id == mid_id) found_removed = 1;
        count++;
        curr = curr->next_task;
    }

    if (count == 2 && !found_removed)
        printf("  [PASS] Removed ID=%lu, list now has 2 nodes\n", (unsigned long)mid_id);
    else {
        printf("  [FAIL] Remove middle failed (count=%d, still_present=%d)\n", count, found_removed);
        pass = 0;
    }

    print_list("After remove mid", test_list);
    printf("\n");

    // ------------------------------------------------------------------
    // TEST 4: remove_task_from_list — remove head node
    // ------------------------------------------------------------------
    printf("--- TEST 4: remove_task_from_list (head node) ---\n");

    uint32_t head_id = test_list->task.task_id;
    test_list = remove_task_from_list(&test_list, head_id);

    count = 0;
    found_removed = 0;
    curr = test_list;
    while (curr != NULL) {
        if (curr->task.task_id == head_id) found_removed = 1;
        count++;
        curr = curr->next_task;
    }

    if (count == 1 && !found_removed)
        printf("  [PASS] Removed head ID=%lu, list now has 1 node\n", (unsigned long)head_id);
    else {
        printf("  [FAIL] Remove head failed (count=%d, still_present=%d)\n", count, found_removed);
        pass = 0;
    }

    print_list("After remove head", test_list);
    printf("\n");

    // ------------------------------------------------------------------
    // TEST 5: remove_task_from_list — remove last node, list becomes NULL
    // ------------------------------------------------------------------
    printf("--- TEST 5: remove_task_from_list (last node -> empty list) ---\n");

    uint32_t last_id = test_list->task.task_id;
    test_list = remove_task_from_list(&test_list, last_id);

    if (test_list == NULL)
        printf("  [PASS] List is NULL after removing last node\n");
    else {
        printf("  [FAIL] List should be NULL but is not\n");
        pass = 0;
    }

    print_list("After remove last", test_list);
    printf("\n");

    // ------------------------------------------------------------------
    // TEST 6: remove_task_from_list — remove from empty list (no crash)
    // ------------------------------------------------------------------
    printf("--- TEST 6: remove_task_from_list (empty list, no crash) ---\n");

    test_list = remove_task_from_list(&test_list, 99);

    if (test_list == NULL)
        printf("  [PASS] Returned NULL gracefully on empty list\n");
    else {
        printf("  [FAIL] Should return NULL on empty list\n");
        pass = 0;
    }

    printf("\n");

    // ------------------------------------------------------------------
    // TEST 7: merge_sort_task_list — single node (no crash, returns itself)
    // ------------------------------------------------------------------
    printf("--- TEST 7: merge_sort_task_list (single node) ---\n");

    add_task_to_list(&test_list, task2);
    test_list = merge_sort_task_list(test_list);

    if (test_list != NULL &&
        test_list->task.task_id == task2.task_id &&
        test_list->next_task == NULL)
        printf("  [PASS] Single-node sort returned correct node\n");
    else {
        printf("  [FAIL] Single-node sort failed\n");
        pass = 0;
    }

    free_list(&test_list);
    printf("\n");

    // ------------------------------------------------------------------
    // BONUS TEST 8: merge_sort_task_list — larger list, all unique deadlines
    // Simulates second hyper-period releases: T1@1000, T2@1000, T3@1500,
    // plus an aperiodic task at deadline 300. Expected sorted order:
    // 300 -> 1000 -> 1000 -> 1500
    // ------------------------------------------------------------------
    printf("--- TEST 8: merge_sort_task_list (4 nodes, mixed deadlines) ---\n");

    dd_task aperiodic = { NULL, APERIODIC, 10, 0, pdMS_TO_TICKS(300),  0 };
    dd_task t1b       = { NULL, PERIODIC,  11, 0, pdMS_TO_TICKS(1000), 0 };
    dd_task t2b       = { NULL, PERIODIC,  12, 0, pdMS_TO_TICKS(1000), 0 };
    dd_task t3b       = { NULL, PERIODIC,  13, 0, pdMS_TO_TICKS(1500), 0 };

    // Add in worst-case reverse order
    add_task_to_list(&test_list, t3b);
    add_task_to_list(&test_list, t2b);
    add_task_to_list(&test_list, t1b);
    add_task_to_list(&test_list, aperiodic);

    print_list("Before sort", test_list);

    test_list = merge_sort_task_list(test_list);

    print_list("After sort ", test_list);

    int bonus_pass = 1;
    uint32_t expected_dl[] = { 300, 1000, 1000, 1500 };
    curr = test_list;
    for (int i = 0; i < 4; i++) {
        if (curr == NULL || curr->task.absolute_deadline != pdMS_TO_TICKS(expected_dl[i])) {
            bonus_pass = 0;
            break;
        }
        curr = curr->next_task;
    }

    if (bonus_pass)
        printf("  [PASS] 4-node sort correct: 300 -> 1000 -> 1000 -> 1500\n");
    else {
        printf("  [FAIL] 4-node sort order incorrect\n");
        pass = 0;
    }

    free_list(&test_list);
    printf("\n");

    // ------------------------------------------------------------------
    // Summary
    // ------------------------------------------------------------------
    printf("========================================\n");
    if (pass)
        printf("  ALL TESTS PASSED\n");
    else
        printf("  ONE OR MORE TESTS FAILED — see above\n");
    printf("========================================\n\n");

    return pass ? 0 : 1;
}
