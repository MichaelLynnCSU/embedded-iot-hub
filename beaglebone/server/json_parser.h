/******************************************************************************
 * \file json_parser.h
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 2026-05-10
 *
 * \brief JSON parsing interface for sensor server.
 *
 * \details Owns JSON frame parsing and SensorData population.
 *          Split from sensor_server.c (2026-05-10).
 ******************************************************************************/

#ifndef JSON_PARSER_H
#define JSON_PARSER_H

/******************************************************************************
 * \brief Parse a JSON body string into SensorData and write to pipe.
 *
 * \param p_json_body - Null-terminated JSON string to parse.
 *                      Must point into a stable buffer -- never pass a
 *                      pointer into the active UART accumulation buffer.
 *
 * \return void
 ******************************************************************************/
void process_json(const char *p_json_body);

#endif /* JSON_PARSER_H */
