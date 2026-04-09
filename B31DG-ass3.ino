#include <stdint.h>
#include <inttypes.h>   // ✅ FIX: required for PRIu32, PRIi64
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "workkernel.h"

// ─── Pin definitions ────────────────────────────────────────────────
#define IN_A      32
#define IN_B      33
#define IN_SYNC   14
#define IN_S      27
#define IN_MODE    1

#define ACK_A     22
#define ACK_B     21
#define ACK_AGG   19
#define ACK_C     18
#define ACK_D     17
#define ACK_S     16
#define ACK_SYNC  4
#define PIN_ERR   2

// ─── Periods ────────────────────────────────────────────────────────
#define PERIOD_A    pdMS_TO_TICKS(10)
#define PERIOD_B    pdMS_TO_TICKS(20)
#define PERIOD_C    pdMS_TO_TICKS(50)
#define PERIOD_D   pdMS_TO_TICKS(50)
#define PERIOD_S    pdMS_TO_TICKS(30)

#define FINAL_REPORT_AFTER_SECONDS 10   // ✅ FIX: added

// ─── Shared tokens ──────────────────────────────────────────────────
volatile uint32_t tokenA   = 0;
volatile uint32_t tokenB   = 0;
volatile uint32_t tokenAGG = 0;
volatile uint32_t tokenC   = 0;
volatile uint32_t tokenD   = 0;
volatile uint32_t tokenS   = 0;

// ─── Edge counters ─────────────────────────────────────────────────
volatile uint32_t countA = 0;
volatile uint32_t countB = 0;

// ─── Task counters ─────────────────────────────────────────────────
uint32_t IDA = 1, IDB = 1, IDAGG = 1, IDC = 1, IDD = 1, IDS = 1;

volatile uint32_t startMonitoringTime = 0;

// ─── Semaphores ────────────────────────────────────────────────────
static SemaphoreHandle_t semSyncStart = NULL;
static SemaphoreHandle_t semAforAGG   = NULL;
static SemaphoreHandle_t semBforAGG   = NULL;
static SemaphoreHandle_t semS         = NULL;
static SemaphoreHandle_t tokenMutex   = NULL;

// ===== Monitoring class =====
class TimingMonitor {
 public:
  void setPeriodicReportEverySeconds(uint32_t seconds) {
    periodic_report_every_us_ = (uint64_t)seconds * 1000000ull;
    next_periodic_report_us_ = (periodic_report_every_us_ > 0u) ? (t0_ + periodic_report_every_us_) : 0u;
  }
  void setFinalReportAfterSeconds(uint32_t seconds) {
    final_report_after_us_ = (uint64_t)seconds * 1000000ull;
    final_report_deadline_us_ = (final_report_after_us_ > 0u) ? (t0_ + final_report_after_us_) : 0u;
    final_report_printed_ = false;
  }

  // Call once, after SYNCH is released
  void synch() {
    t0_ = micros();
    resetTask(a_);
    resetTask(b_);
    resetTask(agg_);
    resetTask(c_);
    resetTask(d_);
    resetTask(s_);
    s_q_head_ = 0;
    s_q_tail_ = 0;
    s_q_count_ = 0;
    next_periodic_report_us_ =
        (periodic_report_every_us_ > 0u) ? (t0_ + periodic_report_every_us_) : 0u;
    final_report_deadline_us_ = (final_report_after_us_ > 0u) ? (t0_ + final_report_after_us_) : 0u;
    final_report_printed_ = false;
  }

  // CALL TO NOTIFY EVERY RELEASE OF S
  void notifySRelease() {
    const uint64_t t = micros();
    if (s_q_count_ < S_RELEASE_Q_MAX) {
      s_release_us_[s_q_tail_] = t;
      s_q_tail_ = (s_q_tail_ + 1u) % S_RELEASE_Q_MAX;
      s_q_count_++;
    }
  }

  // CALL AT THE START AND END OF TASK EXECUTIONS
  void beginTaskA(uint32_t id) { beginTask(a_, id, t0_ + (uint64_t)id * 10000ull); }
  void endTaskA() { endTask(a_); }
  void beginTaskB(uint32_t id) { beginTask(b_, id, t0_ + (uint64_t)id * 20000ull); }
  void endTaskB() { endTask(b_); }
  void beginTaskAGG(uint32_t id) { beginTask(agg_, id, t0_ + (uint64_t)id * 20000ull); }
  void endTaskAGG() { endTask(agg_); }
  void beginTaskC(uint32_t id) { beginTask(c_, id, t0_ + (uint64_t)id * 50000ull); }
  void endTaskC() { endTask(c_); }
  void beginTaskD(uint32_t id) { beginTask(d_, id, t0_ + (uint64_t)id * 50000ull); }
  void endTaskD() { endTask(d_); }
  void beginTaskS(uint32_t id) { beginTask(s_, id, popSRelease()); }
  void endTaskS() { endTask(s_); }

  // returns true if all the deadlines have been met so far
  bool allDeadlinesMet() const {
    return a_.misses == 0 && b_.misses == 0 && agg_.misses == 0 && c_.misses == 0 &&
           d_.misses == 0 && s_.misses == 0;
  }

  // check if a report must be printed
  bool pollReports() {
    const uint64_t now = micros();
    if (periodic_report_every_us_ > 0u && now >= next_periodic_report_us_) {
      const uint64_t periods_missed =
          (now - next_periodic_report_us_) / periodic_report_every_us_;
      next_periodic_report_us_ += (periods_missed + 1u) * periodic_report_every_us_;
      report();
    }
    if (!final_report_printed_ && final_report_deadline_us_ > 0u &&
        now >= final_report_deadline_us_) {
      printFinalReport();
      final_report_printed_ = true;
      return true;
    }
    return false;
  }

  // Print task report
  void report() const {
    Serial.printf("[MON] T0=%llu us\n", t0_);
    reportOne("A", a_);
    reportOne("B", b_);
    reportOne("AGG", agg_);
    reportOne("C", c_);
    reportOne("D", d_);
    reportOne("S", s_);
  }

  void printFinalReport() const {
    Serial.println("FINAL_REPORT_BEGIN");
    report();
    Serial.println("FINAL_REPORT_END");
  }

 private:
  struct TaskStats {
    uint32_t jobs = 0;
    uint32_t misses = 0;
    uint32_t id = 0;
    bool active = false;
    uint64_t start_us = 0;
    uint64_t release_us = 0;
    uint64_t max_exec_us = 0;
    int64_t worst_lateness_us = 0;
    uint64_t deadline_us = 0;
  };

  TaskStats a_{0, 0, 0, false, 0, 0, 0, 0, 10000};
  TaskStats b_{0, 0, 0, false, 0, 0, 0, 0, 20000};
  TaskStats agg_{0, 0, 0, false, 0, 0, 0, 0, 20000};
  TaskStats c_{0, 0, 0, false, 0, 0, 0, 0, 50000};
  TaskStats d_{0, 0, 0, false, 0, 0, 0, 0, 50000};
  TaskStats s_{0, 0, 0, false, 0, 0, 0, 0, 30000};
  uint64_t t0_ = 0;
  uint64_t periodic_report_every_us_ = 0u;
  uint64_t final_report_after_us_ = 0u;
  uint64_t next_periodic_report_us_ = 0u;
  uint64_t final_report_deadline_us_ = 0u;
  bool final_report_printed_ = false;
  static const uint32_t S_RELEASE_Q_MAX = 32u;
  uint64_t s_release_us_[S_RELEASE_Q_MAX]{};
  uint32_t s_q_head_ = 0;
  uint32_t s_q_tail_ = 0;
  uint32_t s_q_count_ = 0;

  static void resetTask(TaskStats &t) {
    t.jobs = 0;
    t.misses = 0;
    t.id = 0;
    t.active = false;
    t.start_us = 0;
    t.release_us = 0;
    t.max_exec_us = 0;
    t.worst_lateness_us = 0;
  }

  static void beginTask(TaskStats &t, uint32_t id, uint64_t release_us) {
    t.active = true;
    t.id = id;
    t.release_us = release_us;
    t.start_us = micros();
  }

  static void endTask(TaskStats &t) {
    if (!t.active) {
      return;
    }
    const uint64_t end_us = micros();
    const uint64_t exec_us = end_us - t.start_us;
    const uint64_t abs_deadline_us = t.release_us + t.deadline_us;
    const int64_t lateness_us = (int64_t)end_us - (int64_t)abs_deadline_us;
    if (exec_us > t.max_exec_us) {
      t.max_exec_us = exec_us;
    }
    if (lateness_us > t.worst_lateness_us) {
      t.worst_lateness_us = lateness_us;
    }
    t.jobs++;
    if (lateness_us > 0) {
      t.misses++;
    }
    t.active = false;
  }

  uint64_t popSRelease() {
    if (s_q_count_ == 0u) {
      return micros();
    }
    const uint64_t t = s_release_us_[s_q_head_];
    s_q_head_ = (s_q_head_ + 1u) % S_RELEASE_Q_MAX;
    s_q_count_--;
    return t;
  }

  static void reportOne(const char *name, const TaskStats &t) {
    Serial.printf("[MON] %s jobs=%" PRIu32 " misses=%" PRIu32 " max_exec=%lluus worst_late=%" PRIi64
                  "us\n",
                  name, t.jobs, t.misses, t.max_exec_us, t.worst_lateness_us);
  }
};




// ✅ FIX: create global instance
TimingMonitor g_monitor;


// ─── Atomic counter protection ─────────────────────────────────────
portMUX_TYPE muxCounter = portMUX_INITIALIZER_UNLOCKED;

// ─── Sporadic tracking ─────────────────────────────────────────────
volatile TickType_t sArrivalTick = 0;

// ─── ISRs ──────────────────────────────────────────────────────────
void IRAM_ATTR countRisingEdges_A() { countA++; }
void IRAM_ATTR countRisingEdges_B() { countB++; }

void IRAM_ATTR handle_SYNC() {
    static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    static bool fired = false;

    if (!fired) {
        fired = true;

        startMonitoringTime = xTaskGetTickCountFromISR(); // ✅ FIX

        g_monitor.synch();

        xSemaphoreGiveFromISR(semSyncStart, &xHigherPriorityTaskWoken);

        digitalWrite(ACK_SYNC, HIGH);

        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void IRAM_ATTR handle_S() {
    TickType_t now = xTaskGetTickCountFromISR();

    if ((TickType_t)(now - sArrivalTick) < pdMS_TO_TICKS(30)) return;

    sArrivalTick = now;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    xSemaphoreGiveFromISR(semS, &xHigherPriorityTaskWoken);

    g_monitor.notifySRelease();   // ✅ FIX

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// ─── Helper ────────────────────────────────────────────────────────
static uint32_t edgesInLastWindow(volatile uint32_t &counter) {
    uint32_t val;
    taskENTER_CRITICAL(&muxCounter);
    val = counter;
    counter = 0;
    taskEXIT_CRITICAL(&muxCounter);
    return val;
}

// ─── Task A ────────────────────────────────────────────────────────
void vTaskA(void *pvParameters) {
    xSemaphoreTake(semSyncStart, portMAX_DELAY);
    xSemaphoreGive(semSyncStart);

    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        uint32_t count_A = edgesInLastWindow(countA);
        uint32_t seed = (IDA << 16) ^ count_A ^ 0xA1;

        g_monitor.beginTaskA(IDA);

        digitalWrite(ACK_A, HIGH);

        uint32_t result = WorkKernel(672000, seed);

        xSemaphoreTake(tokenMutex, portMAX_DELAY);
        tokenA = result;                 // ✅ FIX (protected write)
        xSemaphoreGive(tokenMutex);

        digitalWrite(ACK_A, LOW);

        g_monitor.endTaskA();

        Serial.printf("A,%u,%u,%u\n", IDA, count_A, tokenA);

        IDA++;

        xSemaphoreGive(semAforAGG);

        vTaskDelayUntil(&xLastWakeTime, PERIOD_A);
    }
}

// ─── Task B ────────────────────────────────────────────────────────
void vTaskB(void *pvParameters) {
    xSemaphoreTake(semSyncStart, portMAX_DELAY);
    xSemaphoreGive(semSyncStart);

    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        uint32_t count_B = edgesInLastWindow(countB);
        uint32_t seed = (IDB << 16) ^ count_B ^ 0xB2;

        g_monitor.beginTaskB(IDB);

        digitalWrite(ACK_B, HIGH);

        uint32_t result = WorkKernel(672000, seed);

        xSemaphoreTake(tokenMutex, portMAX_DELAY);
        tokenB = result;                 // ✅ FIX
        xSemaphoreGive(tokenMutex);

        digitalWrite(ACK_B, LOW);

        g_monitor.endTaskB();

        Serial.printf("B,%u,%u,%u\n", IDB, count_B, tokenB);

        IDB++;

        xSemaphoreGive(semBforAGG);

        vTaskDelayUntil(&xLastWakeTime, PERIOD_B);
    }
}

// Task AGG – period 20 ms, priority 3 
void vTaskAGG(void *pvParameters) { 
    xSemaphoreTake(semSyncStart, portMAX_DELAY); 
    xSemaphoreGive(semSyncStart); 
    for (;;) { 
        xSemaphoreTake(semAforAGG, portMAX_DELAY);
         xSemaphoreTake(semBforAGG, portMAX_DELAY);
          uint32_t localA, localB;
           xSemaphoreTake(tokenMutex, portMAX_DELAY);
            localA = tokenA;
             localB = tokenB; 
             xSemaphoreGive(tokenMutex);
              uint32_t agg = (localA != 0 && localB != 0) ? (localA ^ localB) : 0xDEADBEEF; uint32_t seed = (IDAGG << 16) ^ agg ^ 0xD4; 
              g_monitor.beginTaskAGG(IDAGG); digitalWrite(ACK_AGG, HIGH); tokenAGG = WorkKernel(480000, seed); digitalWrite(ACK_AGG, LOW); g_monitor.endTaskAGG();
               Serial.printf("AGG,%u,%u,%u\n", IDAGG, agg, tokenAGG); 
               IDAGG++; 
        } 
} 

// Task C – period 50 ms, priority 2
void vTaskC(void *pvParameters) {
    xSemaphoreTake(semSyncStart, portMAX_DELAY);
    xSemaphoreGive(semSyncStart);

    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        if (IN_MODE == 1) {
            uint32_t seedC = (IDC << 16) ^ 0xC3;

            g_monitor.beginTaskC(IDC);

            digitalWrite(ACK_C, HIGH);

            uint32_t result = WorkKernel(1680000, seedC);

            xSemaphoreTake(tokenMutex, portMAX_DELAY);
            tokenC = result;                      // ✅ protected write
            xSemaphoreGive(tokenMutex);

            digitalWrite(ACK_C, LOW);

            g_monitor.endTaskC();

            Serial.printf("C,%u,%u\n", IDC, tokenC);

            IDC++;
        }

        vTaskDelayUntil(&xLastWakeTime, PERIOD_C);
    }
}

// Task D – period 50 ms, priority 2
void vTaskD(void *pvParameters) {
    xSemaphoreTake(semSyncStart, portMAX_DELAY);
    xSemaphoreGive(semSyncStart);

    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        if (IN_MODE == 1) {
            uint32_t seedD = (IDD << 16) ^ 0xD5;

            g_monitor.beginTaskD(IDD);

            digitalWrite(ACK_D, HIGH);

            uint32_t result = WorkKernel(960000, seedD);

            xSemaphoreTake(tokenMutex, portMAX_DELAY);
            tokenD = result;                      // ✅ protected write
            xSemaphoreGive(tokenMutex);

            digitalWrite(ACK_D, LOW);

            g_monitor.endTaskD();

            Serial.printf("D,%u,%u\n", IDD, tokenD);

            IDD++;
        }

        vTaskDelayUntil(&xLastWakeTime, PERIOD_D);
    }
}

    // Task S – sporadic, triggered by ISR, deadline 30 ms, priority 5 
    void vTaskS(void *pvParameters) { 
        for (;;) {
             if (xSemaphoreTake(semS, portMAX_DELAY) == pdTRUE) {
                 TickType_t now = xTaskGetTickCount(); 
                 TickType_t elapsed = now - sArrivalTick; 
                 if (elapsed > (TickType_t)PERIOD_S) { 
                    digitalWrite(PIN_ERR, HIGH); 
                    vTaskDelay(pdMS_TO_TICKS(1)); 
                    digitalWrite(PIN_ERR, LOW); 
                    continue; 
                    } 
                    uint32_t seed = (IDS << 16) ^ 0x55;
                     g_monitor.beginTaskS(IDS);
                      digitalWrite(ACK_S, HIGH); 
                      tokenS = WorkKernel(600000, seed); 
                      digitalWrite(ACK_S, LOW);
                       g_monitor.endTaskS(); 
                       Serial.printf("S,%u,%u\n", IDS, tokenS); IDS++; 
                       } 
                    } 
    }

    void vPrintReport(void *pvParameters) {

    for (;;) {   // ✅ FIX: must loop

        if (startMonitoringTime != 0) {

            TickType_t now = xTaskGetTickCount();

            if ((now - startMonitoringTime) >= pdMS_TO_TICKS(FINAL_REPORT_AFTER_SECONDS * 1000)) {

                if (!g_monitor.allDeadlinesMet()) {
                    digitalWrite(PIN_ERR, HIGH);
                }

                if (g_monitor.pollReports()) {
                    while (true) {
                        asm volatile("nop"); // stop system
                    }
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // ✅ prevent CPU hogging
    }
}

void setup() {
    Serial.begin(115200);

    g_monitor.setPeriodicReportEverySeconds(0);
    g_monitor.setFinalReportAfterSeconds(FINAL_REPORT_AFTER_SECONDS);

    pinMode(IN_A, INPUT);
    pinMode(IN_B, INPUT);
    pinMode(IN_SYNC, INPUT_PULLUP);
    pinMode(IN_S, INPUT_PULLUP);

    const uint8_t outPins[] = {
        ACK_A, ACK_B, ACK_AGG, ACK_C, ACK_D,
        ACK_S, ACK_SYNC, PIN_ERR
    };

    for (uint8_t p : outPins) {
        pinMode(p, OUTPUT);
        digitalWrite(p, LOW);
    }

    semSyncStart = xSemaphoreCreateBinary();
    semAforAGG   = xSemaphoreCreateBinary();
    semBforAGG   = xSemaphoreCreateBinary();
    semS         = xSemaphoreCreateBinary();
    tokenMutex   = xSemaphoreCreateMutex();

    attachInterrupt(digitalPinToInterrupt(IN_A), countRisingEdges_A, RISING);
    attachInterrupt(digitalPinToInterrupt(IN_B), countRisingEdges_B, RISING);
    attachInterrupt(digitalPinToInterrupt(IN_SYNC), handle_SYNC, RISING);
    attachInterrupt(digitalPinToInterrupt(IN_S), handle_S, FALLING);

    xTaskCreatePinnedToCore(vTaskA, "TaskA", 2048, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(vTaskB, "TaskB", 2048, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(vTaskAGG, "TaskAGG", 2048, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(vTaskC, "TaskC", 2048, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(vTaskD, "TaskD", 2048, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(vTaskS, "TaskS", 2048, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(vPrintReport, "printReport", 2048, NULL, 5, NULL, 1);

    Serial.println("waiting for SYNC signal...");
}

void loop() {
    vTaskDelay(portMAX_DELAY);
}