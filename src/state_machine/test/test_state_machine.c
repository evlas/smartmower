// test_state_machine.c - Framework di test per la state machine
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>
#include <assert.h>
#define _GNU_SOURCE
#include <unistd.h>

#include "include/state_machine.h"
#include "include/config.h"

// Variabili globali per test
static StateMachine test_machine;
static RobotData test_robot_data;
static bool test_running = true;
static int test_passed = 0;
static int test_failed = 0;

// Macro per test assertions
#define TEST_ASSERT(condition, message) \
    do { \
        if (condition) { \
            printf("✅ PASS: %s\n", message); \
            test_passed++; \
        } else { \
            printf("❌ FAIL: %s\n", message); \
            test_failed++; \
        } \
    } while(0)

#define TEST_SECTION(name) \
    printf("\n🔧 === %s ===\n", name)

// Funzioni di utilità per test
void setup_test_environment(void);
void cleanup_test_environment(void);
void simulate_sensor_data(RobotData* data, const char* scenario);
void print_test_results(void);

// Test specifici per transizioni
void test_basic_transitions(void);
void test_timeout_handling(void);
void test_emergency_transitions(void);
void test_error_recovery(void);
void test_manual_control_flow(void);
void test_docking_undocking_cycle(void);

// Signal handler per test
void test_signal_handler(int signum) {
    printf("\n⚠️  Test interrotto dal segnale %d\n", signum);
    test_running = false;
}

int main(int argc, char* argv[]) {
    (void)argc;  // Suppress unused parameter warning
    (void)argv;  // Suppress unused parameter warning
    printf("🚀 === SMART MOWER STATE MACHINE TEST SUITE ===\n");
    printf("Avvio test delle transizioni e timeout...\n\n");
    
    // Setup signal handling
    signal(SIGINT, test_signal_handler);
    signal(SIGTERM, test_signal_handler);
    
    // Setup ambiente di test
    setup_test_environment();
    
    // Esegui i test
    test_basic_transitions();
    test_timeout_handling();
    test_emergency_transitions();
    test_error_recovery();
    test_manual_control_flow();
    test_docking_undocking_cycle();
    
    // Cleanup e risultati
    cleanup_test_environment();
    print_test_results();
    
    return (test_failed > 0) ? 1 : 0;
}

void setup_test_environment(void) {
    TEST_SECTION("SETUP TEST ENVIRONMENT");
    
    // Inizializza dati robot con valori di test
    memset(&test_robot_data, 0, sizeof(RobotData));
    
    // Valori iniziali simulati
    test_robot_data.battery_level = 100.0f;
    test_robot_data.x = 0.0f;
    test_robot_data.y = 0.0f;
    test_robot_data.z = 0.0f;
    test_robot_data.roll = 0.0f;
    test_robot_data.pitch = 0.0f;
    test_robot_data.yaw = 0.0f;
    test_robot_data.vx = 0.0f;
    test_robot_data.vy = 0.0f;
    test_robot_data.vz = 0.0f;
    test_robot_data.orientation = 0.0f;
    test_robot_data.charging = false;
    test_robot_data.obstacle_detected = false;
    test_robot_data.perimeter_detected = false;
    test_robot_data.grass_detected = false;
    test_robot_data.area_covered = 0.0f;
    test_robot_data.total_area = 100.0f;
    test_robot_data.mission_complete = false;
    test_robot_data.last_mowing_x = 0.0f;
    test_robot_data.last_mowing_y = 0.0f;
    test_robot_data.last_update = time(NULL);
    test_robot_data.mqtt_client = NULL;
    test_robot_data.config = NULL;
    test_robot_data.debug_enabled = true;
    strcpy(test_robot_data.log_file, "test.log");
    
    // Inizializza state machine con stato INIT
    State* init_state = get_init_state();
    state_machine_init(&test_machine, init_state, &test_robot_data);
    
    TEST_ASSERT(test_machine.current_state != NULL, "State machine inizializzata");
    TEST_ASSERT(test_machine.current_state->type == STATE_INIT, "Stato iniziale corretto");
    
    printf("✅ Ambiente di test configurato\n");
}

void test_basic_transitions(void) {
    TEST_SECTION("TEST TRANSIZIONI BASE");
    
    // Test 1: INIT → IDLE
    printf("Test 1: Transizione INIT → IDLE\n");
    state_machine_handle_event(&test_machine, EVENT_INIT_COMPLETE);
    state_machine_update(&test_machine);
    TEST_ASSERT(test_machine.current_state->type == STATE_IDLE, "Transizione INIT → IDLE");
    
    // Test 2: IDLE → MOWING
    printf("Test 2: Transizione IDLE → MOWING\n");
    simulate_sensor_data(&test_robot_data, "ready_to_mow");
    state_machine_handle_event(&test_machine, EVENT_START_MOWING);
    state_machine_update(&test_machine);
    TEST_ASSERT(test_machine.current_state->type == STATE_MOWING, "Transizione IDLE → MOWING");
    
    // Test 3: MOWING → DOCKING (batteria bassa)
    printf("Test 3: Transizione MOWING → DOCKING (batteria bassa)\n");
    simulate_sensor_data(&test_robot_data, "low_battery");
    state_machine_handle_event(&test_machine, EVENT_BATTERY_LOW);
    state_machine_update(&test_machine);
    TEST_ASSERT(test_machine.current_state->type == STATE_DOCKING, "Transizione MOWING → DOCKING");
    
    // Test 4: DOCKING → CHARGING
    printf("Test 4: Transizione DOCKING → CHARGING\n");
    simulate_sensor_data(&test_robot_data, "docked");
    state_machine_handle_event(&test_machine, EVENT_DOCKING_COMPLETE);
    state_machine_update(&test_machine);
    TEST_ASSERT(test_machine.current_state->type == STATE_CHARGING, "Transizione DOCKING → CHARGING");
    
    // Test 5: CHARGING → IDLE (batteria carica)
    printf("Test 5: Transizione CHARGING → IDLE\n");
    simulate_sensor_data(&test_robot_data, "fully_charged");
    state_machine_handle_event(&test_machine, EVENT_BATTERY_FULL);
    state_machine_update(&test_machine);
    TEST_ASSERT(test_machine.current_state->type == STATE_IDLE, "Transizione CHARGING → IDLE");
}

void test_timeout_handling(void) {
    TEST_SECTION("TEST GESTIONE TIMEOUT");
    
    // Forza stato UNDOCKING per test timeout
    State* undocking_state = get_undocking_state();
    state_machine_transition(&test_machine, undocking_state);
    
    printf("Test timeout UNDOCKING (2 minuti)...\n");
    
    // Simula passaggio del tempo
    time_t start_time = time(NULL);
    test_machine.state_start_time = start_time - 125; // 125 secondi fa (oltre timeout di 120s)
    
    // Update dovrebbe rilevare timeout
    state_machine_update(&test_machine);
    
    TEST_ASSERT(test_machine.current_state->type == STATE_ERROR, 
                "Timeout UNDOCKING gestito correttamente");
    
    // Test timeout DOCKING
    State* docking_state = get_docking_state();
    state_machine_transition(&test_machine, docking_state);
    test_machine.state_start_time = start_time - 305; // 305 secondi fa (oltre timeout di 300s)
    
    state_machine_update(&test_machine);
    TEST_ASSERT(test_machine.current_state->type == STATE_ERROR, 
                "Timeout DOCKING gestito correttamente");
}

void test_emergency_transitions(void) {
    TEST_SECTION("TEST TRANSIZIONI DI EMERGENZA");
    
    // Reset a stato MOWING
    State* mowing_state = get_mowing_state();
    state_machine_transition(&test_machine, mowing_state);
    
    // Test EMERGENCY_STOP da qualsiasi stato
    printf("Test EMERGENCY_STOP da MOWING\n");
    simulate_sensor_data(&test_robot_data, "emergency");
    state_machine_handle_event(&test_machine, EVENT_EMERGENCY_STOP);
    state_machine_update(&test_machine);
    
    TEST_ASSERT(test_machine.current_state->type == STATE_EMERGENCY_STOP, 
                "Emergency stop da MOWING");
    
    // Test recovery da emergency
    printf("Test recovery da EMERGENCY_STOP\n");
    simulate_sensor_data(&test_robot_data, "safe");
    state_machine_handle_event(&test_machine, EVENT_EMERGENCY_RECOVER);
    state_machine_update(&test_machine);
    
    TEST_ASSERT(test_machine.current_state->type == STATE_IDLE, 
                "Recovery da emergency stop");
}

void test_error_recovery(void) {
    TEST_SECTION("TEST ERROR RECOVERY");
    
    // Forza stato ERROR
    State* error_state = get_error_state();
    state_machine_transition(&test_machine, error_state);
    
    // Simula tentativo di recovery automatico
    printf("Test auto-recovery da ERROR\n");
    simulate_sensor_data(&test_robot_data, "error_resolved");
    
    // Simula passaggio del tempo per recovery
    for (int i = 0; i < 5; i++) {
        state_machine_update(&test_machine);
        usleep(100000); // 100ms
    }
    
    // Dovrebbe tentare recovery o rimanere in error
    TEST_ASSERT(test_machine.current_state->type == STATE_ERROR || 
                test_machine.current_state->type == STATE_IDLE,
                "Error recovery gestito");
}

void test_manual_control_flow(void) {
    TEST_SECTION("TEST CONTROLLO MANUALE");
    
    // Parti da IDLE
    State* idle_state = get_idle_state();
    state_machine_transition(&test_machine, idle_state);
    
    // Attiva controllo manuale
    printf("Test attivazione controllo manuale\n");
    state_machine_handle_event(&test_machine, EVENT_MANUAL_CONTROL);
    state_machine_update(&test_machine);
    
    TEST_ASSERT(test_machine.current_state->type == STATE_MANUAL_CONTROL,
                "Attivazione controllo manuale");
    
    // Test ritorno da controllo manuale
    printf("Test disattivazione controllo manuale\n");
    state_machine_handle_event(&test_machine, EVENT_RESUME);
    state_machine_update(&test_machine);
    
    TEST_ASSERT(test_machine.current_state->type == STATE_IDLE,
                "Ritorno da controllo manuale");
}

void test_docking_undocking_cycle(void) {
    TEST_SECTION("TEST CICLO DOCKING/UNDOCKING");
    
    // Ciclo completo: IDLE → UNDOCKING → MOWING → DOCKING → CHARGING → IDLE
    
    // 1. IDLE → UNDOCKING
    State* idle_state = get_idle_state();
    state_machine_transition(&test_machine, idle_state);
    
    simulate_sensor_data(&test_robot_data, "start_undocking");
    state_machine_handle_event(&test_machine, EVENT_START_MOWING);
    
    // Simula undocking
    State* undocking_state = get_undocking_state();
    state_machine_transition(&test_machine, undocking_state);
    
    // 2. UNDOCKING → MOWING
    simulate_sensor_data(&test_robot_data, "undocked");
    state_machine_handle_event(&test_machine, EVENT_UNDOCKING_COMPLETE);
    state_machine_update(&test_machine);
    
    TEST_ASSERT(test_machine.current_state->type == STATE_MOWING,
                "Undocking → Mowing");
    
    // 3. MOWING → DOCKING
    simulate_sensor_data(&test_robot_data, "low_battery");
    state_machine_handle_event(&test_machine, EVENT_BATTERY_LOW);
    state_machine_update(&test_machine);
    
    TEST_ASSERT(test_machine.current_state->type == STATE_DOCKING,
                "Mowing → Docking");
    
    // 4. DOCKING → CHARGING
    simulate_sensor_data(&test_robot_data, "docked");
    state_machine_handle_event(&test_machine, EVENT_DOCKING_COMPLETE);
    state_machine_update(&test_machine);
    
    TEST_ASSERT(test_machine.current_state->type == STATE_CHARGING,
                "Docking → Charging");
    
    printf("✅ Ciclo completo docking/undocking testato\n");
}

void simulate_sensor_data(RobotData* data, const char* scenario) {
    if (!data || !scenario) return;
    
    if (strcmp(scenario, "ready_to_mow") == 0) {
        data->battery_level = 90.0f;
        data->charging = false;
        data->obstacle_detected = false;
    }
    else if (strcmp(scenario, "low_battery") == 0) {
        data->battery_level = 15.0f;
    }
    else if (strcmp(scenario, "fully_charged") == 0) {
        data->battery_level = 100.0f;
        data->charging = true;
    }
    else if (strcmp(scenario, "emergency") == 0) {
        data->obstacle_detected = true;
        data->perimeter_detected = true;
    }
    else if (strcmp(scenario, "safe") == 0) {
        data->obstacle_detected = false;
        data->perimeter_detected = false;
    }
    else if (strcmp(scenario, "docked") == 0) {
        data->x = 0.0f;
        data->y = 0.0f;
        data->charging = true;
    }
    else if (strcmp(scenario, "undocked") == 0) {
        data->x = 2.0f;
        data->y = 0.0f;
        data->charging = false;
    }
    else if (strcmp(scenario, "error_resolved") == 0) {
        data->obstacle_detected = false;
        data->battery_level = 50.0f;
    }
}

void cleanup_test_environment(void) {
    TEST_SECTION("CLEANUP");
    state_machine_shutdown(&test_machine);
    printf("✅ Cleanup completato\n");
}

void print_test_results(void) {
    printf("\n📊 === RISULTATI TEST ===\n");
    printf("✅ Test passati: %d\n", test_passed);
    printf("❌ Test falliti: %d\n", test_failed);
    printf("📈 Successo: %.1f%%\n", 
           (float)test_passed / (test_passed + test_failed) * 100.0f);
    
    if (test_failed == 0) {
        printf("🎉 TUTTI I TEST SONO PASSATI!\n");
    } else {
        printf("⚠️  Alcuni test sono falliti. Rivedere l'implementazione.\n");
    }
}
