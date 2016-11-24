#include <arch_helpers.h>
#include <assert.h>
#include <bl_common.h>
#include <debug.h>
#include <gic_v2.h>
#include <gic_v3.h>
#include <interrupt_mgmt.h>
#include <platform.h>
#include <stdint.h>
#include <platform_def.h>
#include <fiq_smp_call.h>
#include <stdio.h>  //for printf

extern void irq_raise_softirq(unsigned int map, unsigned int irq);
volatile struct call_function_data cfd[PLATFORM_CORE_COUNT];

int fiq_smp_call_function(unsigned int map, inter_cpu_call_func_t func, void *info, int wait)
{
	int cpu;
	int lockval, tmp;

	if (!func) {
		printf("inter-cpu call is failed due to invalid func\n");
		return -1;
	}

	for (cpu = 0; cpu < PLATFORM_CORE_COUNT; cpu++) {
		if (map & (1 << cpu)) {
			printf("Wait until cpu %d is ready for inter-cpu call\n", cpu);

			__asm__ volatile(
				"1: ldxr  %w0, [%2]\n"
				" add %w0, %w0, %w3\n"
				" stxr  %w1, %w0, [%2]\n"
				" cbnz  %w1, 1b"
					: "=&r" (lockval),  "=&r" (tmp) : "r" (&(cfd[cpu].lock)), "Ir" (1): "cc");

			cfd[cpu].func = func;
			cfd[cpu].info = info;
		}
	}

	printf("Send SGI to cpu (map: 0x%x) for inter-cpu call\n", map);
	irq_raise_softirq(map, FIQ_SMP_CALL_SGI);

	if (wait) {
		for (cpu = 0; cpu < PLATFORM_CORE_COUNT; cpu++) {
			if (map & (1 << cpu)) {
				printf("Wait until cpu %d is done\n", cpu);

				while (cfd[cpu].lock != 0)
					;

				printf("cpu %d is done\n", cpu);
			}
		}
	}

	return 0;
}

void fiq_icc_isr(void)
{
	int cpu;
	int lockval, tmp;
	uint64_t mpidr;

	printf("inter-cpu-call interrupt is triggered\n");

	mpidr = read_mpidr();
	cpu = platform_get_core_pos(mpidr);

	if ((cfd[cpu].func != NULL) && (cfd[cpu].lock == 1)) {
		cfd[cpu].func(cfd[cpu].info);
		cfd[cpu].func = NULL;
		cfd[cpu].info = NULL;

		/* free lock */
		__asm__ volatile(
		"1:	ldxr	%w0, [%2]\n"
		"	sub	%w0, %w0, %w3\n"
		"	stxr	%w1, %w0, [%2]\n"
		"	cbnz	%w1, 1b"
			: "=&r" (lockval), "=&r" (tmp)
			: "r" (&(cfd[cpu].lock)), "Ir" (1)
			: "cc");
	} else {
		printf("cfd[%d] is invalid (func = 0x%lx, lock = %d)\n", cpu,
			(unsigned long) cfd[cpu].func, cfd[cpu].lock);
	}

	printf("CPU_%d cfd lock = %d\n", cpu, cfd[cpu].lock);
}
