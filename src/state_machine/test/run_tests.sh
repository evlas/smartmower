#!/bin/bash

# Script di automazione test per Smart Mower State Machine
# Uso: ./run_tests.sh [opzioni]

set -e  # Exit on error

# Colori per output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Funzioni di utilità
print_header() {
    echo -e "${BLUE}================================${NC}"
    echo -e "${BLUE}$1${NC}"
    echo -e "${BLUE}================================${NC}"
}

print_success() {
    echo -e "${GREEN}✅ $1${NC}"
}

print_warning() {
    echo -e "${YELLOW}⚠️  $1${NC}"
}

print_error() {
    echo -e "${RED}❌ $1${NC}"
}

print_info() {
    echo -e "${BLUE}ℹ️  $1${NC}"
}

# Variabili
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TEST_LOG="test_results.log"
VERBOSE=false
QUICK=false
CLEAN_ONLY=false

# Parsing argomenti
while [[ $# -gt 0 ]]; do
    case $1 in
        -v|--verbose)
            VERBOSE=true
            shift
            ;;
        -q|--quick)
            QUICK=true
            shift
            ;;
        -c|--clean)
            CLEAN_ONLY=true
            shift
            ;;
        -h|--help)
            echo "Uso: $0 [opzioni]"
            echo "Opzioni:"
            echo "  -v, --verbose    Output dettagliato"
            echo "  -q, --quick      Test rapidi (30s timeout)"
            echo "  -c, --clean      Solo pulizia, no test"
            echo "  -h, --help       Mostra questo aiuto"
            exit 0
            ;;
        *)
            print_error "Opzione sconosciuta: $1"
            exit 1
            ;;
    esac
done

# Cambia directory di lavoro
cd "$SCRIPT_DIR"

print_header "SMART MOWER STATE MACHINE TEST SUITE"

# Cleanup se richiesto
if [ "$CLEAN_ONLY" = true ]; then
    print_info "Esecuzione cleanup..."
    make -f Makefile.test clean
    print_success "Cleanup completato"
    exit 0
fi

# Verifica prerequisiti
print_info "Verifica prerequisiti..."

# Controlla se le dipendenze sono installate
if ! pkg-config --exists libmosquitto; then
    print_error "libmosquitto non trovata. Installare con: sudo apt-get install libmosquitto-dev"
    exit 1
fi

if ! pkg-config --exists json-c; then
    print_error "json-c non trovata. Installare con: sudo apt-get install libjson-c-dev"
    exit 1
fi

print_success "Prerequisiti verificati"

# Cleanup precedente
print_info "Pulizia file precedenti..."
make -f Makefile.test clean > /dev/null 2>&1 || true

# Compilazione
print_info "Compilazione test suite..."
if make -f Makefile.test all > build.log 2>&1; then
    print_success "Compilazione completata"
else
    print_error "Errore di compilazione"
    echo "Log di compilazione:"
    cat build.log
    exit 1
fi

# Preparazione ambiente test
print_info "Preparazione ambiente di test..."

# Crea file di configurazione di test se non esiste
if [ ! -f "config_test.json" ]; then
    cat > config_test.json << EOF
{
    "robot": {
        "name": "SmartMower Test",
        "id": "test_001"
    },
    "mqtt": {
        "host": "localhost",
        "port": 1883,
        "client_id": "smartmower_test",
        "topics": {
            "status": "smartmower/test/status",
            "command": "smartmower/test/command",
            "telemetry": "smartmower/test/telemetry"
        }
    },
    "safety": {
        "emergency_stop_timeout": 5,
        "max_tilt_angle": 30,
        "max_current": 10.0
    },
    "navigation": {
        "max_speed": 1.0,
        "turn_speed": 0.5,
        "obstacle_distance": 0.3
    },
    "battery": {
        "low_threshold": 20.0,
        "critical_threshold": 10.0,
        "full_threshold": 95.0
    }
}
EOF
    print_info "File di configurazione test creato"
fi

# Esecuzione test
print_header "ESECUZIONE TEST"

# Avvia broker MQTT locale se non è già in esecuzione
if ! pgrep -x "mosquitto" > /dev/null; then
    print_info "Avvio broker MQTT locale per test..."
    mosquitto -d -p 1883 > /dev/null 2>&1 || print_warning "Impossibile avviare mosquitto"
fi

# Esegui i test
TEST_START_TIME=$(date +%s)

if [ "$VERBOSE" = true ]; then
    print_info "Esecuzione test in modalità verbose..."
    if DEBUG_CONFIG=1 ./test_state_machine | tee "$TEST_LOG"; then
        TEST_RESULT=0
    else
        TEST_RESULT=$?
    fi
elif [ "$QUICK" = true ]; then
    print_info "Esecuzione test rapidi (30s timeout)..."
    if timeout 30s ./test_state_machine | tee "$TEST_LOG"; then
        TEST_RESULT=0
    else
        TEST_RESULT=$?
        if [ $TEST_RESULT -eq 124 ]; then
            print_warning "Test interrotti per timeout (30s)"
        fi
    fi
else
    print_info "Esecuzione test standard..."
    if ./test_state_machine | tee "$TEST_LOG"; then
        TEST_RESULT=0
    else
        TEST_RESULT=$?
    fi
fi

TEST_END_TIME=$(date +%s)
TEST_DURATION=$((TEST_END_TIME - TEST_START_TIME))

# Analisi risultati
print_header "ANALISI RISULTATI"

if [ -f "$TEST_LOG" ]; then
    PASSED_TESTS=$(grep -c "✅ PASS:" "$TEST_LOG" 2>/dev/null || echo "0")
    FAILED_TESTS=$(grep -c "❌ FAIL:" "$TEST_LOG" 2>/dev/null || echo "0")
    TOTAL_TESTS=$((PASSED_TESTS + FAILED_TESTS))
    
    echo "📊 Statistiche Test:"
    echo "   • Test totali: $TOTAL_TESTS"
    echo "   • Test passati: $PASSED_TESTS"
    echo "   • Test falliti: $FAILED_TESTS"
    echo "   • Durata: ${TEST_DURATION}s"
    
    if [ $FAILED_TESTS -eq 0 ] && [ $TOTAL_TESTS -gt 0 ]; then
        print_success "TUTTI I TEST SONO PASSATI! 🎉"
        SUCCESS_RATE="100%"
    elif [ $TOTAL_TESTS -gt 0 ]; then
        SUCCESS_RATE=$(echo "scale=1; $PASSED_TESTS * 100 / $TOTAL_TESTS" | bc -l 2>/dev/null || echo "N/A")
        print_warning "Alcuni test sono falliti (Successo: ${SUCCESS_RATE}%)"
    else
        print_error "Nessun test eseguito o rilevato"
    fi
else
    print_error "File di log dei test non trovato"
fi

# Generazione report
print_info "Generazione report..."

REPORT_FILE="test_report_$(date +%Y%m%d_%H%M%S).txt"
cat > "$REPORT_FILE" << EOF
SMART MOWER STATE MACHINE - TEST REPORT
=======================================
Data: $(date)
Durata test: ${TEST_DURATION}s
Modalità: $([ "$VERBOSE" = true ] && echo "Verbose" || ([ "$QUICK" = true ] && echo "Quick" || echo "Standard"))

RISULTATI:
- Test totali: $TOTAL_TESTS
- Test passati: $PASSED_TESTS  
- Test falliti: $FAILED_TESTS
- Tasso di successo: ${SUCCESS_RATE:-N/A}

DETTAGLI:
$(cat "$TEST_LOG" 2>/dev/null || echo "Log non disponibile")

SISTEMA:
- OS: $(uname -a)
- Compilatore: $(gcc --version | head -n1)
- Dipendenze: mosquitto $(mosquitto --help 2>&1 | head -n1 | grep -o 'version [0-9.]*' || echo "N/A")

EOF

print_success "Report salvato in: $REPORT_FILE"

# Cleanup finale
if [ "$VERBOSE" != true ]; then
    rm -f build.log
fi

# Exit con codice appropriato
if [ $TEST_RESULT -eq 0 ]; then
    print_success "Test suite completata con successo"
    exit 0
else
    print_error "Test suite fallita (codice: $TEST_RESULT)"
    exit $TEST_RESULT
fi
