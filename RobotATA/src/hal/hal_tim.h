
#ifndef HAL_TIM_H
#define HAL_TIM_H

#include <stdint.h>

typedef enum
{
	HAL_TIM_1,  /*!< Advanced-control timer TIM1 */
	HAL_TIM_2,  /*!< General-purpose timer TIM2 */
	HAL_TIM_3,  /*!< General-purpose timer TIM3 */
	HAL_TIM_4,  /*!< General-purpose timer TIM4 */
	HAL_TIM_5,  /*!< General-purpose timer TIM5 */
	HAL_TIM_6,  /*!< Basic timer TIM6 */
	HAL_TIM_7,  /*!< Basic timer TIM7 */
	HAL_TIM_8,  /*!< Advanced-control timer TIM8 */
	HAL_TIM_9,  /*!< General-purpose timer TIM9 */
	HAL_TIM_10, /*!< General-purpose timer TIM10 */
	HAL_TIM_11, /*!< General-purpose timer TIM11 */
	HAL_TIM_12, /*!< General-purpose timer TIM12 */
	HAL_TIM_13, /*!< General-purpose timer TIM13 */
	HAL_TIM_14, /*!< General-purpose timer TIM14 */
	HAL_TIM_END
} hal_tim_list_t;

typedef struct hal_tim_t
{
	hal_tim_list_t  tim;
	uint32_t        us;
	void            (*clbk)(struct hal_tim_t *obj);
} hal_tim_t;

void hal_tim_init(hal_tim_t *obj, hal_tim_list_t tim);

void hal_tim_set_clbk(hal_tim_t *obj, void (*clbk)(hal_tim_t *obj));

void hal_tim_set_us(hal_tim_t *obj, uint32_t us);

void hal_tim_start_once(hal_tim_t *obj);

void hal_tim_start_cyclic(hal_tim_t *obj);

void hal_tim_stop(hal_tim_t *obj);

#endif
