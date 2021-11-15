/**
 * @file Moving_Average_Filter.h
 *
 * @brief Implementation of simple moving average filter.
 *
 * @author Goce Boshkovski
 * @date 12.11 2021
 *
 * @copyright GNU General Public License v3
 *
 */

#ifndef MOVING_AVERAGE_FILTER_H
#define MOVING_ACERAGE_FILTER_H

#include <vector>

#include "setila/bdc/CircularBuffer.h"

/** \defgroup filters */

/** \defgroup simple_moving_average_filter_group Simple Moving Average FIlter
 *  \ingroup filters
 * @{
 */


/**
 * \class Moving_Average_Filter
 *
 * @brief Simple moving average filter implementation using circular buffer as a moving window.
 *
 * Can be used on real-time data (readings from a sensor) or applied on a vector of
 * already sampled data.
 *
 * During the initial filling of the circular buffer the sampling window size
 * is equal to the data-set size. All the values in the moving window
 * are equally weighted.
 *
 * @tparam tElement represents a sensor reading as a filter input
 *
 * @example moving_average_window.cpp
 */
template <class tElement>
class Moving_Average_Filter
{
private:
	tElement m_sum;				/**< Sum of all elements in the moving window */
	tElement m_last_output;		/**< The last output of the filter */
	CircularBuffer<tElement> *m_buffer = nullptr;	/**< Internal circular buffer representing the moving window */

	Moving_Average_Filter() = default;

public:
	/**
	 * @brief Constructor. Allocates the buffer for the moving window
	 *
	 * @param[in] window_size the size of the moving window
	 */
	Moving_Average_Filter(unsigned int window_size) {
		m_buffer = new CircularBuffer<tElement>(window_size);
		m_sum = 0;
	}

	~Moving_Average_Filter() {
		if (m_buffer != nullptr) {
			delete m_buffer;
		}
	}

	/**
	 * @brief Calculates on the fly the output of the filter as a mean value for the elements in the window.
	 *
	 * During the initial filling of the circular buffer the sampling window size is equal to the data-set size.
	 * When there are enough samples for filling the circular buffer, the full window size is used for
	 * calculating the average value of the samples in the window.
	 *
	 * @param[in] input the current value of the input sample
	 *
	 * @return tElement the filters output calculated as described above
	 *
	 */
	tElement output(const tElement &input) {
		if (m_buffer->current_number_of_elements() < m_buffer->size()) {
			m_sum += input;
		}
		else {
			m_sum += (input - m_buffer->get_head());
		}

		m_buffer->addElement(input);

		m_last_output = m_sum / m_buffer->current_number_of_elements();
		return m_last_output;
	}

	/**
	 * @brief Applies the filter on a vector of data.
	 *
	 * @param[in] input pointer to a vector with samples
	 * @param[out] output pointer to a vector values after applying the filter
	 */
	void filter(const std::vector<tElement> *input, std::vector<tElement> *output)
	{
		if ((input->size() != output->size()) && (output->size() != 0)) {
			return;
		}

		unsigned int i = 0;
		for(i = 0; i < input->size(); i++)
		{
			if (m_buffer->current_number_of_elements() < m_buffer->size()) {
				m_sum += input->at(i);
			}
			else {
				m_sum += (input->at(i) - m_buffer->get_head());
			}

			m_buffer->addElement(input->at(i));

			output->at(i) = m_sum / m_buffer->current_number_of_elements();
		}

		m_last_output = output->at(i - 1);
	}


	/**
	 * @brief Auxiliary function. Returns the last output value of the filter
	 *
	 * @return
	 */
	const tElement last_output() const {
		return m_last_output;
	}

};

/** @} */

#endif
