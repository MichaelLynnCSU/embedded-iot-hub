#ifndef INCLUDE_EVENT_AGGREGATOR_H_
#define INCLUDE_EVENT_AGGREGATOR_H_

/******************************************************************************
 * \brief Start the event aggregator task.
 *
 * \details Registers a bus subscriber with EVT_ALL_MASK and spawns the
 *          aggregator task. Call from app_main after bus_init() and all
 *          bus_register_subscriber() calls.
 ******************************************************************************/
void event_aggregator_start(void);

#endif /* INCLUDE_EVENT_AGGREGATOR_H_ */
