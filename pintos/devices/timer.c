#include "devices/timer.h"
#include <debug.h>
#include <inttypes.h>
#include <round.h>
#include <stdio.h>
#include "threads/interrupt.h"
#include "threads/io.h"
#include "threads/synch.h"
#include "threads/thread.h"

/* See [8254] for hardware details of the 8254 timer chip. */
#if TIMER_FREQ < 19
#error 8254 timer requires TIMER_FREQ >= 19
#endif
#if TIMER_FREQ > 1000
#error TIMER_FREQ <= 1000 recommended
#endif

/* Number of timer ticks since OS booted. */
//OS가 부팅된 이후 경과된 타이머 틱 수를 누적.
static int64_t ticks;

/* Number of loops per timer tick.
   Initialized by timer_calibrate(). */
   //타이머 틱당 루프 수.
static unsigned loops_per_tick;


static intr_handler_func timer_interrupt;
static bool too_many_loops (unsigned loops);
static void busy_wait (int64_t loops);
static void real_time_sleep (int64_t num, int32_t denom);

/* Sets up the 8254 Programmable Interval Timer (PIT) to
   interrupt PIT_FREQ times per second, and registers the
   corresponding interrupt. */
   //8254 프로그래머블 인터벌 타이머(PIT)를 설정하여 초당 PIT_FREQ번 인터럽트가 발생하도록 하고, 해당 인터럽트를 등록
void
timer_init (void) {
	/* 8254 input frequency divided by TIMER_FREQ, rounded to
	   nearest. */
	   //8254 입력 주파수를 TIMER_FREQ로 나눈 값, 가장 가까운 정수로 반올림
	   //초당 틱 수 계산, 설정
	   //하드웨어 칩이 몇 번 진동할 때마다 한 번씩 틱(인터럽트)을 울릴 지를 계산
	uint16_t count = (1193180 + TIMER_FREQ / 2) / TIMER_FREQ;

	//ex)0x43 : 포트번호
	outb (0x43, 0x34);    /* CW: counter 0, LSB then MSB, mode 2, binary. */
	outb (0x40, count & 0xff);
	outb (0x40, count >> 8);

	intr_register_ext (0x20, timer_interrupt, "8254 Timer");
}

/* Calibrates loops_per_tick, used to implement brief delays. */
//부팅 시 CPU 속도를 측정하여 아주 짧은 시간을 지연시킬 때 사용할 기준 값(loops_per_tick)을 자동으로 계산
void
timer_calibrate (void) {
	unsigned high_bit, test_bit;

	ASSERT (intr_get_level () == INTR_ON);
	printf ("Calibrating timer...  ");

	/* Approximate loops_per_tick as the largest power-of-two
	   still less than one timer tick. */
	   //loops_per_tick를 1 타이머 틱보다 작은 가장 큰 2의 거듭제곱으로 근사화
	   //1틱 동안 CPU가 몇 번의 루프를 수행할 수 있는지 측정하여 loops_per_tick을 계산
	loops_per_tick = 1u << 10;
	while (!too_many_loops (loops_per_tick << 1)) {
		loops_per_tick <<= 1;
		ASSERT (loops_per_tick != 0);
	}

	/* Refine the next 8 bits of loops_per_tick. */
	//loops_per_tick의 다음 8비트를 정밀하게 조정
	high_bit = loops_per_tick;
	for (test_bit = high_bit >> 1; test_bit != high_bit >> 10; test_bit >>= 1)
		if (!too_many_loops (high_bit | test_bit))
			loops_per_tick |= test_bit;

	printf ("%'"PRIu64" loops/s.\n", (uint64_t) loops_per_tick * TIMER_FREQ);
}

/* Returns the number of timer ticks since the OS booted. */
// OS가 부팅된 이후의 타이머 틱 수를 반환
int64_t
timer_ticks (void) {
	enum intr_level old_level = intr_disable ();
	int64_t t = ticks;
	intr_set_level (old_level);
	barrier ();
	return t;
}

/* Returns the number of timer ticks elapsed since THEN, which
   should be a value once returned by timer_ticks(). */
   //THEN 이후 경과된 타이머 틱 수를 반환. THEN은 timer_ticks()에 의해 반환된 값이어야 함
   //timer_elapsed(then): 특정 과거 시점(then)으로부터 현재까지 몇 틱이 지났는지 계산
   //timer_ticks(): 누적 틱 값을 안전하게 읽어오기 위해 잠시 인터럽트를 껐다 켜며 ticks를 반환   
int64_t
timer_elapsed (int64_t then) {
	return timer_ticks () - then;
}

/* Suspends execution for approximately TICKS timer ticks. */
//지정한 타이머 틱(TICKS) 동안 대략적으로 실행을 일시 중단
//alarm-negative.c, alarm-zero.c 테스트 케이스
//alarm clock기능 구현.
//음수거나 0일 때 예외처리 기능 구현
//busy waiting이 아닌 특정 틱 수 동안 슬립 리스트에서 잠들게 하게
void
timer_sleep (int64_t ticks) {
	int64_t start = timer_ticks ();

	if(ticks <= 0){	//음수나 0일 때 예외처리
		return;
	}

	ASSERT (intr_get_level () == INTR_ON);

    int64_t wakeup_tick = start + ticks;
    thread_sleep (wakeup_tick);	
}


/* Suspends execution for approximately MS milliseconds. */
//대략 MS 밀리초(millisecond) 동안 실행을 일시 중단
//필요한 시간단위를 골라쓰기 위함
void
timer_msleep (int64_t ms) {
	real_time_sleep (ms, 1000);
}

/* Suspends execution for approximately US microseconds. */
//대략 US 마이크로초(microsecond) 동안 실행을 일시 중단
void
timer_usleep (int64_t us) {
	real_time_sleep (us, 1000 * 1000);
}

/* Suspends execution for approximately NS nanoseconds. */
//대략 NS 나노초(nanosecond) 동안 실행을 일시 중단
void
timer_nsleep (int64_t ns) {
	real_time_sleep (ns, 1000 * 1000 * 1000);
}

/* Prints timer statistics. */
//타이머 통계 출력
void
timer_print_stats (void) {
	printf ("Timer: %"PRId64" ticks\n", timer_ticks ());
}

/* Timer interrupt handler. */
//구현할 부분
//틱마다 확인해 이 안에서 슬립 리스트를 확인하여 시간이 된 스레드를 찾아 thread_unblock()으로 깨워주는 기능 구현
//thread_unblock()을 호출해 정확하게 깨워주기만 해도 우선순위가 높은 스레드가 먼저 깨워지기에 코드구현 없어도 alarm-priority.c 통과 가능
//하드웨어 타이머 인터럽트 핸들러인(timer_interrupt가 timer_sleep에 잠들어 있는 스레드들을 깨움)
static void
timer_interrupt (struct intr_frame *args UNUSED) {
	ticks++;
	thread_tick ();

	thread_awake (ticks);
}

/* Returns true if LOOPS iterations waits for more than one timer
   tick, otherwise false. */
   //LOOPS 번의 반복이 1 타이머 틱보다 오래 걸리면 true를, 아니면 false를 반환
static bool
too_many_loops (unsigned loops) {
	/* Wait for a timer tick. */
	//타이머 틱이 바뀔 때까지 기다림
	int64_t start = ticks;
	while (ticks == start)
		barrier ();

	/* Run LOOPS loops. */
	//LOOPS만큼 루프만큼 실행
	start = ticks;
	busy_wait (loops);

	/* If the tick count changed, we iterated too long. */
	//틱 수가 바뀌었다면, 루프를 너무 오래 반복한 것임
	barrier ();
	return start != ticks;
}

/* Iterates through a simple loop LOOPS times, for implementing
   brief delays.

   Marked NO_INLINE because code alignment can significantly
   affect timings, so that if this function was inlined
   differently in different places the results would be difficult
   to predict. */
   //간단한 루프를 LOOPS번 반복하여 짧은 지연을 구현
   //컴파일러 최적화로 인라인화될 경우 위치에 따라 타이밍이 크게 달라져 결과를 예측하기 어려워지므로, 
   //NO_INLINE으로 마킹되어 있음
static void NO_INLINE
busy_wait (int64_t loops) {
	while (loops-- > 0)
		barrier ();
}

/* Sleep for approximately NUM/DENOM seconds. */
//대략 NUM/DENOM 초 동안 재움
static void
real_time_sleep (int64_t num, int32_t denom) {
	/* Convert NUM/DENOM seconds into timer ticks, rounding down.
	//NUM/DENOM 초를 타이머 틱으로 변환하며, 내림(버림) 처리

	   (NUM / DENOM) s
	   ---------------------- = NUM * TIMER_FREQ / DENOM ticks.
	   1 s / TIMER_FREQ ticks
	   */
	int64_t ticks = num * TIMER_FREQ / denom;

	ASSERT (intr_get_level () == INTR_ON);
	if (ticks > 0) {
		/* We're waiting for at least one full timer tick.  Use
		   timer_sleep() because it will yield the CPU to other
		   processes. */
		   //적어도 1개 이상의 온전한 타이머 틱을 기다리는 경우
           //다른 프로세스(스레드)에 CPU를 양보할 수 있도록 timer_sleep()을 사용
		timer_sleep (ticks);
	} else {
		/* Otherwise, use a busy-wait loop for more accurate
		   sub-tick timing.  We scale the numerator and denominator
		   down by 1000 to avoid the possibility of overflow. */
		   //그렇지 않다면, 1틱 미만의 더 정밀한 타이밍을 위해 바쁜 대기(busy-wait) 루프를 사용 
           //오버플로우 가능성을 피하기 위해 분자와 분모를 1000으로 스케일 다운
		ASSERT (denom % 1000 == 0);
		busy_wait (loops_per_tick * num / 1000 * TIMER_FREQ / (denom / 1000));
	}
}
