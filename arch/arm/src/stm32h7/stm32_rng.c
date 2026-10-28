/****************************************************************************
 * arch/arm/src/stm32h7/stm32_rng.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <debug.h>
#include <errno.h>
#include <time.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/fs/fs.h>
#include <nuttx/drivers/drivers.h>

#include "hardware/stm32_rng.h"
#include "stm32_rcc.h"
#include "arm_internal.h"

#if defined(CONFIG_STM32H7_RNG)
#if defined(CONFIG_DEV_RANDOM) || defined(CONFIG_DEV_URANDOM_ARCH)

/* Bound initialization reads, including contention, without claiming any
 * physical entropy or real-time qualification for this software deadline.
 */

#define RNG_READ_TIMEOUT_MS 100

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int stm32_rng_initialize(void);
static int stm32_rng_interrupt(int irq, void *context, void *arg);
static void stm32_rng_enable(void);
static void stm32_rng_disable(void);
static ssize_t stm32_rng_read(struct file *filep, char *buffer, size_t);

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rng_dev_s
{
  mutex_t rd_devlock;   /* Threads can only exclusively access the RNG */
  sem_t rd_readsem;     /* To block until the buffer is filled  */
  char *rd_buf;
  size_t rd_buflen;
  uint32_t rd_lastval;
  int rd_result;
  bool rd_first;
  bool rd_initialized;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct rng_dev_s g_rngdev =
{
  .rd_devlock = NXMUTEX_INITIALIZER,
  .rd_readsem = SEM_INITIALIZER(0),
};

static const struct file_operations g_rngops =
{
  NULL,            /* open */
  NULL,            /* close */
  stm32_rng_read,  /* read */
};

/****************************************************************************
 * Private functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_rng_initialize
 ****************************************************************************/

static int stm32_rng_initialize(void)
{
  int ret;

  if (g_rngdev.rd_initialized)
    {
      return OK;
    }

#ifdef CONFIG_STM32H7_HSI48
  if ((getreg32(STM32_RCC_CR) & RCC_CR_HSI48RDY) == 0)
    {
      return -EIO;
    }

  /* RNGSEL=00 selects HSI48; preserve the other peripheral clocks. */

  modifyreg32(STM32_RCC_D2CCIP2R, RCC_D2CCIP2R_RNGSEL_MASK, 0);
#endif

  _info("Initializing RNG\n");

  ret = irq_attach(STM32_IRQ_RNG, stm32_rng_interrupt, NULL);
  if (ret < 0)
    {
      /* We could not attach the ISR to the interrupt */

      _info("Could not attach IRQ.\n");

      return ret;
    }

  /* A caller buffer is installed before enabling either interrupt source. */

  stm32_rng_disable();
  g_rngdev.rd_initialized = true;

  return OK;
}

/****************************************************************************
 * Name: stm32_rng_enable
 ****************************************************************************/

static void stm32_rng_enable(void)
{
  uint32_t regval;

  g_rngdev.rd_first = true;

  regval = getreg32(STM32_RNG_CR);
  regval |= RNG_CR_RNGEN | RNG_CR_IE;
  putreg32(regval, STM32_RNG_CR);
  up_enable_irq(STM32_IRQ_RNG);
}

/****************************************************************************
 * Name: stm32_rng_disable
 ****************************************************************************/

static void stm32_rng_disable(void)
{
  uint32_t regval;
  regval = getreg32(STM32_RNG_CR);
  regval &= ~(RNG_CR_RNGEN | RNG_CR_IE);
  putreg32(regval, STM32_RNG_CR);
  up_disable_irq(STM32_IRQ_RNG);
}

/****************************************************************************
 * Name: stm32_rng_interrupt
 ****************************************************************************/

static int stm32_rng_interrupt(int irq, void *context, void *arg)
{
  uint32_t rngsr;
  uint32_t data;

  if (g_rngdev.rd_buf == NULL || g_rngdev.rd_buflen == 0)
    {
      stm32_rng_disable();
      return OK;
    }

  rngsr = getreg32(STM32_RNG_SR);

  if (rngsr & (RNG_SR_SEIS | RNG_SR_CEIS))
    {
      g_rngdev.rd_result = -EIO;
      goto complete;
    }

  if (!(rngsr & RNG_SR_DRDY))
    {
      /* No value yet.  The caller's original deadline remains in force. */

      return OK;
    }

  data = getreg32(STM32_RNG_DR);

  /* As required by the FIPS PUB (Federal Information Processing Standard
   * Publication) 140-2, the first random number generated after setting the
   * RNGEN bit should not be used, but saved for comparison with the next
   * generated random number. Each subsequent generated random number has to
   * be compared with the previously generated number. The test fails if any
   * two compared numbers are equal (continuous random number generator
   * test).
   */

  if (g_rngdev.rd_first)
    {
      g_rngdev.rd_first = false;
      g_rngdev.rd_lastval = data;
      return OK;
    }

  if (g_rngdev.rd_lastval == data)
    {
      g_rngdev.rd_result = -EIO;
      goto complete;
    }

  /* If we get here, the random number is valid. */

  g_rngdev.rd_lastval = data;

  if (g_rngdev.rd_buflen >= 4)
    {
      g_rngdev.rd_buflen -= 4;
      memcpy(&g_rngdev.rd_buf[g_rngdev.rd_buflen], &data, sizeof(data));
    }
  else
    {
      while (g_rngdev.rd_buflen > 0)
        {
          g_rngdev.rd_buf[--g_rngdev.rd_buflen] = (char)data;
          data >>= 8;
        }
    }

  if (g_rngdev.rd_buflen == 0)
    {
      goto complete;
    }

  return OK;

complete:
  /* Revoke the pointer before waking its owner.  A pending interrupt can
   * never write into a completed, interrupted or timed-out caller buffer.
   */

  stm32_rng_disable();
  g_rngdev.rd_buf = NULL;
  g_rngdev.rd_buflen = 0;
  nxsem_post(&g_rngdev.rd_readsem);
  return OK;
}

/****************************************************************************
 * Name: stm32_rng_read
 ****************************************************************************/

static ssize_t stm32_rng_read(struct file *filep, char *buffer,
                              size_t buflen)
{
  const struct timespec interval =
    {
      .tv_sec = 0,
      .tv_nsec = RNG_READ_TIMEOUT_MS * NSEC_PER_MSEC
    };
  struct timespec deadline;
  struct timespec now;
  irqstate_t flags;
  int ret;

  if (buflen == 0)
    {
      return 0;
    }

  if (buffer == NULL || !g_rngdev.rd_initialized)
    {
      return -EINVAL;
    }

  clock_systime_timespec(&deadline);
  clock_timespec_add(&deadline, &interval, &deadline);
  ret = nxmutex_clocklock(&g_rngdev.rd_devlock, CLOCK_MONOTONIC, &deadline);
  if (ret < 0)
    {
      return ret;
    }

  clock_systime_timespec(&now);
  if (clock_timespec_compare(&now, &deadline) >= 0)
    {
      nxmutex_unlock(&g_rngdev.rd_devlock);
      return -ETIMEDOUT;
    }

  flags = enter_critical_section();

  /* Reset the operation semaphore with 0 for blocking until the
   * buffer is filled from interrupts.
   */

  nxsem_reset(&g_rngdev.rd_readsem, 0);

  g_rngdev.rd_buflen = buflen;
  g_rngdev.rd_buf = buffer;
  g_rngdev.rd_result = 0;

  /* Enable RNG with interrupts */

  stm32_rng_enable();
  leave_critical_section(flags);

  /* Wait until the buffer is filled */

  ret = nxsem_clockwait(&g_rngdev.rd_readsem, CLOCK_MONOTONIC, &deadline);

  flags = enter_critical_section();
  stm32_rng_disable();

  if (ret >= 0)
    {
      ret = g_rngdev.rd_result;
      clock_systime_timespec(&now);
      if (clock_timespec_compare(&now, &deadline) >= 0)
        {
          ret = -ETIMEDOUT;
        }
      else if (g_rngdev.rd_buflen != 0)
        {
          ret = -EIO;
        }
    }

  g_rngdev.rd_buf = NULL;
  g_rngdev.rd_buflen = 0;
  leave_critical_section(flags);

  nxmutex_unlock(&g_rngdev.rd_devlock);
  return ret < 0 ? ret : (ssize_t)buflen;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: devrandom_register
 *
 * Description:
 *   Initialize the RNG hardware and register the /dev/random driver.
 *   Must be called BEFORE devurandom_register.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_DEV_RANDOM
void devrandom_register(void)
{
  if (stm32_rng_initialize() == OK)
    {
      register_driver("/dev/random", &g_rngops, 0444, NULL);
    }
}
#endif

/****************************************************************************
 * Name: devurandom_register
 *
 * Description:
 *   Register /dev/urandom
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_DEV_URANDOM_ARCH
void devurandom_register(void)
{
  if (stm32_rng_initialize() == OK)
    {
      register_driver("/dev/urandom", &g_rngops, 0444, NULL);
    }
}
#endif

#endif /* CONFIG_DEV_RANDOM || CONFIG_DEV_URANDOM_ARCH */
#endif /* CONFIG_STM32H7_RNG */
