// list_test.c
// Standalone laptop build of DD-task list, delete_dd_task, user task,
// and DDS logic tests. No FreeRTOS dependencies.
//
// Compile and run:
//   gcc -Wall -o list_test list_test.c && ./list_test

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>     // clock() for busy-wait timing, nanosleep() for pauses

#ifdef _WIN32
#include <windows.h>
#define sleep_ms(ms) Sleep(ms)
#else
#include <unistd.h>
#define sleep_ms(ms) usleep((ms) * 1000)
#endif

/*-----------------------------------------------------------*/
// FreeRTOS type aliases
/*-----------------------------------------------------------*/

typedef void*    TaskHandle_t;
#define pdMS_TO_TICKS(ms) ((uint32_t)(ms))  // 1000 Hz tick → 1 tick == 1 ms

/*-----------------------------------------------------------*/
// Structs - identical to main.c
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

typedef enum {
    RELEASE_TASK,
    COMPLETE_TASK,
    GET_ACTIVE_LIST,
    GET_COMPLETED_LIST,
    GET_OVERDUE_LIST,
    CHECK_OVERDUE
} msg_type;

typedef struct {
    msg_type type;
    dd_task  task;
} dd_message;

// Stub state - captured by stub_delete_dd_task
static dd_message last_delete_message;
static int        delete_was_called = 0;

/*-----------------------------------------------------------*/
// Forward declarations
/*-----------------------------------------------------------*/

dd_task_list* add_task_to_list(dd_task_list **head, dd_task new_task);
dd_task_list* remove_task_from_list(dd_task_list **head, uint32_t task_id);
dd_task_list* merge_sort_task_list(dd_task_list *head);
dd_message    build_delete_message(uint32_t task_id);
dd_message    build_release_message(TaskHandle_t handle, task_type type,
                                    uint32_t task_id, uint32_t absolute_deadline);
long          simulated_user_task(int exec_ms);
static void   stub_delete_dd_task(uint32_t task_id);

/*-----------------------------------------------------------*/
// Stub functions
/*-----------------------------------------------------------*/

static void stub_delete_dd_task(uint32_t task_id)
{
    dd_task task;
    task.t_handle          = NULL;
    task.type              = PERIODIC;
    task.task_id           = task_id;
    task.release_time      = 0;
    task.absolute_deadline = 0;
    task.completion_time   = 0;

    dd_message message;
    message.type = COMPLETE_TASK;
    message.task = task;

    last_delete_message = message;
    delete_was_called   = 1;
}

dd_message build_delete_message(uint32_t task_id)
{
    dd_task task;
    task.t_handle          = NULL;
    task.type              = PERIODIC;
    task.task_id           = task_id;
    task.release_time      = 0;
    task.absolute_deadline = 0;
    task.completion_time   = 0;

    dd_message message;
    message.type = COMPLETE_TASK;
    message.task = task;
    return message;
}

dd_message build_release_message(TaskHandle_t handle, task_type type,
                                  uint32_t task_id, uint32_t absolute_deadline)
{
    dd_task task;
    task.t_handle          = handle;
    task.type              = type;
    task.task_id           = task_id;
    task.release_time      = 0;
    task.absolute_deadline = absolute_deadline;
    task.completion_time   = 0;

    dd_message message;
    message.type = RELEASE_TASK;
    message.task = task;
    return message;
}

long simulated_user_task(int exec_ms)
{
    clock_t start  = clock();
    clock_t target = start + ((clock_t)exec_ms * CLOCKS_PER_SEC) / 1000;
    while (clock() < target) { }
    clock_t end    = clock();
    return (long)((end - start) * 1000 / CLOCKS_PER_SEC);
}

/*-----------------------------------------------------------*/
// List functions - identical logic to main.c
// pvPortMalloc -> malloc, vPortFree -> free
/*-----------------------------------------------------------*/

dd_task_list* add_task_to_list(dd_task_list **head, dd_task new_task)
{
    dd_task_list *node = (dd_task_list*)malloc(sizeof(dd_task_list));
    if (node == NULL) return *head;

    node->task      = new_task;
    node->next_task = NULL;

    if (*head == NULL) { *head = node; return *head; }

    dd_task_list *curr = *head;
    while (curr->next_task != NULL) curr = curr->next_task;
    curr->next_task = node;
    return *head;
}

dd_task_list* remove_task_from_list(dd_task_list **head, uint32_t task_id)
{
    if (*head == NULL) return NULL;

    dd_task_list *curr = *head;
    dd_task_list *prev = NULL;

    while (curr != NULL)
    {
        if (curr->task.task_id == task_id)
        {
            if (prev == NULL) *head = curr->next_task;
            else              prev->next_task = curr->next_task;
            free(curr);
            return *head;
        }
        prev = curr;
        curr = curr->next_task;
    }
    return *head;
}

dd_task_list* merge_sort_task_list(dd_task_list *head)
{
    if (head == NULL || head->next_task == NULL) return head;

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
            { right = right->next_task; left_len++; }

            int right_len = size;

            while (left_len > 0 && right_len > 0 && right != NULL)
            {
                if (left->task.absolute_deadline <= right->task.absolute_deadline)
                { tail->next_task = left;  left  = left->next_task;  left_len--;  }
                else
                { tail->next_task = right; right = right->next_task; right_len--; }
                tail = tail->next_task;
            }
            while (left_len > 0 && left != right)
            { tail->next_task = left; left = left->next_task; left_len--; tail = tail->next_task; }
            while (right_len > 0 && right != NULL)
            { tail->next_task = right; right = right->next_task; right_len--; tail = tail->next_task; }

            tail->next_task = NULL;
            curr = right;
        }
    }
    return dummy.next_task;
}

/*-----------------------------------------------------------*/
// Helpers
/*-----------------------------------------------------------*/

// Count nodes in a list
static int count_list(dd_task_list *head)
{
    int n = 0;
    while (head) { n++; head = head->next_task; }
    return n;
}

// Print every node in a list with full task detail
static void print_list_detail(const char *label, dd_task_list *head)
{
    printf("  %-12s: ", label);
    if (head == NULL) {
        printf("(empty)\n");
        return;
    }
    printf("\n");
    dd_task_list *curr = head;
    while (curr != NULL) {
        printf("    ID=%-4lu | type=%-9s | release=%-6lu | deadline=%-6lu | completion=%-6lu\n",
               (unsigned long)curr->task.task_id,
               curr->task.type == PERIODIC ? "PERIODIC" : "APERIODIC",
               (unsigned long)curr->task.release_time,
               (unsigned long)curr->task.absolute_deadline,
               (unsigned long)curr->task.completion_time);
        curr = curr->next_task;
    }
}

// Print the monitor-style summary line + full list detail.
// Mirrors exactly what Monitor_Task prints on hardware, then adds detail.
static void print_monitor_state(uint32_t sim_tick,
                                dd_task_list *active,
                                dd_task_list *completed,
                                dd_task_list *overdue)
{
    printf("\n  [MONITOR] t=%lu ms | Active: %d | Completed: %d | Overdue: %d\n",
           (unsigned long)sim_tick,
           count_list(active),
           count_list(completed),
           count_list(overdue));
    print_list_detail("Active",    active);
    print_list_detail("Completed", completed);
    print_list_detail("Overdue",   overdue);
    printf("\n");
}

// Free all nodes in a list
static void free_list(dd_task_list **head)
{
    dd_task_list *curr = *head;
    while (curr != NULL) { dd_task_list *next = curr->next_task; free(curr); curr = next; }
    *head = NULL;
}

// Print a test header
static void test_header(int num, const char *desc)
{
    printf("--- TEST %d: %s ---\n", num, desc);
}

// Print PASS/FAIL and pause 3 seconds
static void test_result(int *pass, int condition, const char *pass_msg, const char *fail_msg)
{
    if (condition)
        printf("  [PASS] %s\n", pass_msg);
    else {
        printf("  [FAIL] %s\n", fail_msg);
        *pass = 0;
    }
}

static void pause_3s(void)
{
    printf("  (pausing 3s...)\n\n");
    sleep_ms(3000);
}

/*-----------------------------------------------------------*/
// main - all tests
/*-----------------------------------------------------------*/

int main(void)
{
    int pass = 1;

    printf("\n========================================\n");
    printf("  DD-Task Scheduler Tests\n");
    printf("  Based on Test Bench #1 (T1=500ms, T2=500ms, T3=750ms)\n");
    printf("========================================\n\n");

    // Shared list state used across DDS logic tests
    dd_task_list *active_list    = NULL;
    dd_task_list *completed_list = NULL;
    dd_task_list *overdue_list   = NULL;
    dd_task_list *curr;
    uint32_t      sim_tick = 0;

    // Three tasks matching Test Bench #1, released at t=0
    dd_task task1 = { NULL, PERIODIC, 1, 0, pdMS_TO_TICKS(500), 0 };
    dd_task task2 = { NULL, PERIODIC, 2, 0, pdMS_TO_TICKS(500), 0 };
    dd_task task3 = { NULL, PERIODIC, 3, 0, pdMS_TO_TICKS(750), 0 };

    // =================================================================
    // SECTION 1: List Functions
    // =================================================================
    printf("========================================\n");
    printf("  SECTION 1: List Functions\n");
    printf("========================================\n\n");

    // -----------------------------------------------------------------
    // TEST 1: add_task_to_list - count and insertion order
    // -----------------------------------------------------------------
    test_header(1, "add_task_to_list - count and insertion order");

    add_task_to_list(&active_list, task3);
    add_task_to_list(&active_list, task1);
    add_task_to_list(&active_list, task2);

    test_result(&pass, count_list(active_list) == 3,
                "List has 3 nodes after 3 adds",
                "Expected 3 nodes");

    test_result(&pass,
                active_list != NULL &&
                active_list->task.task_id == 3 &&
                active_list->next_task->task.task_id == 1 &&
                active_list->next_task->next_task->task.task_id == 2,
                "Insertion order preserved (3 -> 1 -> 2)",
                "Insertion order wrong");

    print_monitor_state(sim_tick, active_list, completed_list, overdue_list);
    pause_3s();

    // -----------------------------------------------------------------
    // TEST 2: merge_sort_task_list - EDF order after sort
    // -----------------------------------------------------------------
    test_header(2, "merge_sort_task_list - EDF sort by deadline");

    active_list = merge_sort_task_list(active_list);

    curr = active_list;
    int sort_ok = (curr && curr->task.absolute_deadline == pdMS_TO_TICKS(500));
    if (sort_ok) { curr = curr->next_task; sort_ok = (curr && curr->task.absolute_deadline == pdMS_TO_TICKS(500)); }
    if (sort_ok) { curr = curr->next_task; sort_ok = (curr && curr->task.absolute_deadline == pdMS_TO_TICKS(750)); }

    test_result(&pass, sort_ok,
                "Sorted: 500 -> 500 -> 750",
                "Sort order incorrect");

    test_result(&pass,
                active_list && active_list->task.absolute_deadline == pdMS_TO_TICKS(500),
                "Head has earliest deadline (EDF invariant holds)",
                "Head does not have earliest deadline");

    print_monitor_state(sim_tick, active_list, completed_list, overdue_list);
    pause_3s();

    // -----------------------------------------------------------------
    // TEST 3: remove_task_from_list - middle node
    // -----------------------------------------------------------------
    test_header(3, "remove_task_from_list - middle node");

    uint32_t mid_id = active_list->next_task->task.task_id;
    active_list = remove_task_from_list(&active_list, mid_id);

    int found = 0;
    curr = active_list; while (curr) { if (curr->task.task_id == mid_id) found = 1; curr = curr->next_task; }
    test_result(&pass, count_list(active_list) == 2 && !found,
                "Middle node removed, list has 2 nodes",
                "Remove middle failed");

    print_monitor_state(sim_tick, active_list, completed_list, overdue_list);
    pause_3s();

    // -----------------------------------------------------------------
    // TEST 4: remove_task_from_list - head node
    // -----------------------------------------------------------------
    test_header(4, "remove_task_from_list - head node");

    uint32_t head_id = active_list->task.task_id;
    active_list = remove_task_from_list(&active_list, head_id);

    found = 0;
    curr = active_list; while (curr) { if (curr->task.task_id == head_id) found = 1; curr = curr->next_task; }
    test_result(&pass, count_list(active_list) == 1 && !found,
                "Head removed, list has 1 node",
                "Remove head failed");

    print_monitor_state(sim_tick, active_list, completed_list, overdue_list);
    pause_3s();

    // -----------------------------------------------------------------
    // TEST 5: remove_task_from_list - last node → empty
    // -----------------------------------------------------------------
    test_header(5, "remove_task_from_list - last node, list becomes NULL");

    uint32_t last_id = active_list->task.task_id;
    active_list = remove_task_from_list(&active_list, last_id);

    test_result(&pass, active_list == NULL,
                "List is NULL after removing last node",
                "List should be NULL but is not");

    print_monitor_state(sim_tick, active_list, completed_list, overdue_list);
    pause_3s();

    // -----------------------------------------------------------------
    // TEST 6: remove_task_from_list - empty list, no crash
    // -----------------------------------------------------------------
    test_header(6, "remove_task_from_list - empty list, graceful return");

    active_list = remove_task_from_list(&active_list, 99);

    test_result(&pass, active_list == NULL,
                "Returned NULL gracefully on empty list",
                "Should return NULL on empty list");

    print_monitor_state(sim_tick, active_list, completed_list, overdue_list);
    pause_3s();

    // -----------------------------------------------------------------
    // TEST 7: merge_sort_task_list - single node
    // -----------------------------------------------------------------
    test_header(7, "merge_sort_task_list - single node, no crash");

    add_task_to_list(&active_list, task2);
    active_list = merge_sort_task_list(active_list);

    test_result(&pass,
                active_list && active_list->task.task_id == task2.task_id && !active_list->next_task,
                "Single-node sort returned correct node",
                "Single-node sort failed");

    print_monitor_state(sim_tick, active_list, completed_list, overdue_list);
    free_list(&active_list);
    pause_3s();

    // -----------------------------------------------------------------
    // TEST 8: merge_sort_task_list - 4 nodes, mixed deadlines
    // -----------------------------------------------------------------
    test_header(8, "merge_sort_task_list - 4 nodes added in reverse deadline order");

    dd_task aperiodic = { NULL, APERIODIC, 10, 0, pdMS_TO_TICKS(300),  0 };
    dd_task t1b       = { NULL, PERIODIC,  11, 0, pdMS_TO_TICKS(1000), 0 };
    dd_task t2b       = { NULL, PERIODIC,  12, 0, pdMS_TO_TICKS(1000), 0 };
    dd_task t3b       = { NULL, PERIODIC,  13, 0, pdMS_TO_TICKS(1500), 0 };

    add_task_to_list(&active_list, t3b);
    add_task_to_list(&active_list, t2b);
    add_task_to_list(&active_list, t1b);
    add_task_to_list(&active_list, aperiodic);

    printf("  Before sort:\n");
    print_list_detail("Active", active_list);

    active_list = merge_sort_task_list(active_list);

    uint32_t expected_dl[] = { 300, 1000, 1000, 1500 };
    int bonus_ok = 1;
    curr = active_list;
    for (int i = 0; i < 4; i++) {
        if (!curr || curr->task.absolute_deadline != pdMS_TO_TICKS(expected_dl[i]))
        { bonus_ok = 0; break; }
        curr = curr->next_task;
    }

    test_result(&pass, bonus_ok,
                "4-node sort correct: 300 -> 1000 -> 1000 -> 1500",
                "4-node sort order incorrect");

    print_monitor_state(sim_tick, active_list, completed_list, overdue_list);
    free_list(&active_list);
    pause_3s();

    // =================================================================
    // SECTION 2: delete_dd_task and User Task Logic
    // =================================================================
    printf("========================================\n");
    printf("  SECTION 2: delete_dd_task and User Task Logic\n");
    printf("========================================\n\n");

    // -----------------------------------------------------------------
    // TEST 9: delete message type is COMPLETE_TASK
    // -----------------------------------------------------------------
    test_header(9, "delete_dd_task - message.type == COMPLETE_TASK");

    dd_message msg = build_delete_message(42);
    test_result(&pass, msg.type == COMPLETE_TASK,
                "message.type == COMPLETE_TASK",
                "message.type is wrong");
    printf("  message.type = %d (COMPLETE_TASK = %d)\n", msg.type, COMPLETE_TASK);
    pause_3s();

    // -----------------------------------------------------------------
    // TEST 10: delete message carries correct task_id
    // -----------------------------------------------------------------
    test_header(10, "delete_dd_task - task_id passes through correctly");

    msg = build_delete_message(99);
    test_result(&pass, msg.task.task_id == 99,
                "message.task.task_id == 99",
                "task_id field wrong");
    printf("  message.task.task_id = %lu\n", (unsigned long)msg.task.task_id);
    pause_3s();

    // -----------------------------------------------------------------
    // TEST 11: delete message completion_time starts at 0
    // -----------------------------------------------------------------
    test_header(11, "delete_dd_task - completion_time == 0 (DDS stamps on receipt)");

    msg = build_delete_message(7);
    test_result(&pass, msg.task.completion_time == 0,
                "completion_time == 0",
                "completion_time should be 0");
    printf("  message.task.completion_time = %lu\n", (unsigned long)msg.task.completion_time);
    pause_3s();

    // -----------------------------------------------------------------
    // TEST 12: task ID handoff via stub
    // -----------------------------------------------------------------
    test_header(12, "task ID handoff - stub_delete_dd_task called with correct ID");

    uint32_t current_task_id = 1001;
    delete_was_called = 0;
    stub_delete_dd_task(current_task_id);

    test_result(&pass, delete_was_called && last_delete_message.task.task_id == current_task_id,
                "delete called with correct task_id=1001",
                "task_id mismatch or delete not called");
    printf("  captured task_id = %lu\n", (unsigned long)last_delete_message.task.task_id);
    pause_3s();

    // -----------------------------------------------------------------
    // TEST 13: T1 busy-wait timing (~95ms)
    // -----------------------------------------------------------------
    test_header(13, "User_Task1 busy-wait duration (~95ms)");

    long elapsed1 = simulated_user_task(95);
    printf("  Elapsed: %ld ms  (expected ~95ms, tolerance 80-110ms)\n", elapsed1);
    test_result(&pass, elapsed1 >= 80 && elapsed1 <= 110,
                "T1 ran within tolerance",
                "T1 duration out of tolerance");
    pause_3s();

    // -----------------------------------------------------------------
    // TEST 14: T2 busy-wait timing (~150ms)
    // -----------------------------------------------------------------
    test_header(14, "User_Task2 busy-wait duration (~150ms)");

    long elapsed2 = simulated_user_task(150);
    printf("  Elapsed: %ld ms  (expected ~150ms, tolerance 135-165ms)\n", elapsed2);
    test_result(&pass, elapsed2 >= 135 && elapsed2 <= 165,
                "T2 ran within tolerance",
                "T2 duration out of tolerance");
    pause_3s();

    // -----------------------------------------------------------------
    // TEST 15: T3 busy-wait timing (~250ms)
    // -----------------------------------------------------------------
    test_header(15, "User_Task3 busy-wait duration (~250ms)");

    long elapsed3 = simulated_user_task(250);
    printf("  Elapsed: %ld ms  (expected ~250ms, tolerance 235-265ms)\n", elapsed3);
    test_result(&pass, elapsed3 >= 235 && elapsed3 <= 265,
                "T3 ran within tolerance",
                "T3 duration out of tolerance");
    pause_3s();

    // =================================================================
    // SECTION 3: DDS Logic (RELEASE, COMPLETE, CHECK_OVERDUE, EDF)
    // =================================================================
    printf("========================================\n");
    printf("  SECTION 3: DDS Logic\n");
    printf("========================================\n\n");

    // -----------------------------------------------------------------
    // TEST 16: RELEASE_TASK - three tasks released at t=0
    // -----------------------------------------------------------------
    test_header(16, "RELEASE_TASK - three tasks added and sorted at t=0");

    sim_tick = 0;

    // Simulate DDS RELEASE_TASK: stamp release_time, add, sort
    task1.release_time = task2.release_time = task3.release_time = sim_tick;
    add_task_to_list(&active_list, task1);
    add_task_to_list(&active_list, task2);
    add_task_to_list(&active_list, task3);
    active_list = merge_sort_task_list(active_list);

    test_result(&pass, count_list(active_list) == 3,
                "Active list has 3 tasks",
                "Active list count wrong");

    test_result(&pass,
                active_list && active_list->task.absolute_deadline == pdMS_TO_TICKS(500),
                "Head has earliest deadline (EDF = 500ms)",
                "Head deadline wrong");

    test_result(&pass, completed_list == NULL && overdue_list == NULL,
                "Completed and overdue lists are empty",
                "Completed or overdue unexpectedly populated");

    print_monitor_state(sim_tick, active_list, completed_list, overdue_list);
    pause_3s();

    // -----------------------------------------------------------------
    // TEST 17: COMPLETE_TASK on time - task 1 completes at t=95ms
    // -----------------------------------------------------------------
    test_header(17, "COMPLETE_TASK (on time) - task 1 at t=95ms");

    sim_tick = pdMS_TO_TICKS(95);

    curr = active_list;
    while (curr) {
        if (curr->task.task_id == 1) {
            curr->task.completion_time = sim_tick;
            if (curr->task.completion_time <= curr->task.absolute_deadline)
                add_task_to_list(&completed_list, curr->task);
            else
                add_task_to_list(&overdue_list, curr->task);
            break;
        }
        curr = curr->next_task;
    }
    remove_task_from_list(&active_list, 1);
    active_list = merge_sort_task_list(active_list);

    test_result(&pass,
                count_list(active_list) == 2 &&
                count_list(completed_list) == 1 &&
                count_list(overdue_list) == 0,
                "Active=2, Completed=1, Overdue=0",
                "List counts wrong after on-time completion");

    test_result(&pass,
                completed_list &&
                completed_list->task.task_id == 1 &&
                completed_list->task.completion_time == pdMS_TO_TICKS(95),
                "Completed task has correct completion_time=95ms",
                "Completed task fields incorrect");

    print_monitor_state(sim_tick, active_list, completed_list, overdue_list);
    pause_3s();

    // -----------------------------------------------------------------
    // TEST 18: COMPLETE_TASK late - task 2 completes at t=600ms
    // -----------------------------------------------------------------
    test_header(18, "COMPLETE_TASK (late) - task 2 at t=600ms (deadline was 500ms)");

    sim_tick = pdMS_TO_TICKS(600);

    curr = active_list;
    while (curr) {
        if (curr->task.task_id == 2) {
            curr->task.completion_time = sim_tick;
            if (curr->task.completion_time <= curr->task.absolute_deadline)
                add_task_to_list(&completed_list, curr->task);
            else
                add_task_to_list(&overdue_list, curr->task);
            break;
        }
        curr = curr->next_task;
    }
    remove_task_from_list(&active_list, 2);
    active_list = merge_sort_task_list(active_list);

    test_result(&pass,
                count_list(active_list) == 1 &&
                count_list(completed_list) == 1 &&
                count_list(overdue_list) == 1,
                "Active=1, Completed=1, Overdue=1",
                "List counts wrong after late completion");

    test_result(&pass,
                overdue_list && overdue_list->task.task_id == 2,
                "Late task 2 correctly placed in overdue list",
                "Late task not in overdue list");

    print_monitor_state(sim_tick, active_list, completed_list, overdue_list);
    pause_3s();

    // -----------------------------------------------------------------
    // TEST 19: CHECK_OVERDUE - active task misses deadline
    // -----------------------------------------------------------------
    test_header(19, "CHECK_OVERDUE - task 4 (deadline 200ms) still active at t=300ms");

    dd_task r4 = { NULL, PERIODIC, 4, 0, pdMS_TO_TICKS(200), 0 };
    add_task_to_list(&active_list, r4);
    active_list = merge_sort_task_list(active_list);

    sim_tick = pdMS_TO_TICKS(300);

    // Simulate DDS CHECK_OVERDUE handler
    dd_task_list *scan = active_list;
    while (scan) {
        dd_task_list *next = scan->next_task;
        if (sim_tick > scan->task.absolute_deadline) {
            scan->task.completion_time = sim_tick;
            add_task_to_list(&overdue_list, scan->task);
            remove_task_from_list(&active_list, scan->task.task_id);
            active_list = merge_sort_task_list(active_list);
        }
        scan = next;
    }

    int found4_overdue = 0, found3_active = 0;
    curr = overdue_list; while (curr) { if (curr->task.task_id == 4) found4_overdue = 1; curr = curr->next_task; }
    curr = active_list;  while (curr) { if (curr->task.task_id == 3) found3_active  = 1; curr = curr->next_task; }

    test_result(&pass, found4_overdue,
                "Task 4 (missed deadline) moved to overdue list",
                "Task 4 not found in overdue list");

    test_result(&pass, found3_active,
                "Task 3 (deadline 750ms) remains in active list",
                "Task 3 incorrectly removed from active list");

    print_monitor_state(sim_tick, active_list, completed_list, overdue_list);
    pause_3s();

    // -----------------------------------------------------------------
    // TEST 20: EDF ordering - head always has earliest deadline
    // -----------------------------------------------------------------
    test_header(20, "EDF ordering - head always has earliest deadline after releases");

    sim_tick = pdMS_TO_TICKS(300);

    dd_task r5 = { NULL, PERIODIC, 5, sim_tick, pdMS_TO_TICKS(900), 0 };
    dd_task r6 = { NULL, PERIODIC, 6, sim_tick, pdMS_TO_TICKS(600), 0 };

    add_task_to_list(&active_list, r5);
    active_list = merge_sort_task_list(active_list);
    printf("  After adding task5 (deadline=900):\n");
    print_list_detail("Active", active_list);

    add_task_to_list(&active_list, r6);
    active_list = merge_sort_task_list(active_list);
    printf("  After adding task6 (deadline=600):\n");
    print_list_detail("Active", active_list);

    // Active now has: task3(750), task5(900), task6(600) → sorted: task6(600), task3(750), task5(900)
    test_result(&pass,
                active_list && active_list->task.absolute_deadline == pdMS_TO_TICKS(600),
                "Head has earliest deadline (600ms) - EDF invariant holds",
                "EDF head deadline wrong");

    print_monitor_state(sim_tick, active_list, completed_list, overdue_list);
    pause_3s();

    // -----------------------------------------------------------------
    // Final cleanup and summary
    // -----------------------------------------------------------------
    free_list(&active_list);
    free_list(&completed_list);
    free_list(&overdue_list);

    printf("========================================\n");
    if (pass)
        printf("  ALL TESTS PASSED\n");
    else
        printf("  ONE OR MORE TESTS FAILED - see above\n");
    printf("========================================\n\n");

    return pass ? 0 : 1;
}
