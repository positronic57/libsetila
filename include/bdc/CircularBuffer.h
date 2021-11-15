/**
 * @file CircularBuffer.h
 *
 * @brief Template class for implementing circular buffer.
 *
 * @author Goce Boshkovski
 * @date 23 Nov 2019
 *
 * @copyright GNU General Public License v3
 *
 */

#ifndef CIRCULARBUFFER_H_
#define CIRCULARBUFFER_H_

#include "bdc.h"

/** \defgroup circular_buffer_bdc_group Circular Buffer
 *  \ingroup bdc_group
 * @{
 */


/** 
 *  @brief A template class for a circular buffer.
 * 
 *  @tparam tElement single element of the data container
 */
template <class tElement>
class CircularBuffer
{
private:
	unsigned int head = 0;					/**< A head of the circular buffer.*/
	
    unsigned int tail = 0;					/**< A tail of the circular buffer. */
	unsigned int length = 32;				/**< A length of the circular buffer. */
	unsigned int number_of_entries = 0;		/**< Number of entries currently stored in the circular buffer. */
	tElement *queue = new tElement[32];		/**< The actual buffer.*/

    /**
     * @brief Default constructor.
     */
    CircularBuffer() = default;

public:

    /**
	 * @brief A constructor.
	 *
	 * The constructor initializes the head and tail values to 0.
	 * It allocates the memory for the buffer itself.
	 *
	 * @param[in] len the length of the buffer
	 */
	explicit CircularBuffer(unsigned int len): length(len), queue(new tElement[length]) {}

	/**
	 * @brief A virtual destructor of the class.
	 *
	 * It deletes the memory allocated for the buffer.
	 */
	virtual ~CircularBuffer() { delete [] queue; }

	/**
	 * @brief A copy constructor of the class.
	 *
	 * @param[in] source A source object for the copy operation.
	 */
	CircularBuffer(const CircularBuffer<tElement> &source)
	{
		head = source.head;
		tail = source.tail;
		length = source.length;
		number_of_entries = source.number_of_entries;

		queue = new tElement[length];

		if (!isEmpty())
			for(unsigned int index = 0; index < length; index++)
				queue[index] = source.queue[index];
	}

	/**
	 * @brief Overloading of the assignment operator.
	 *
	 * @param[in] source A source object for the assignment operator.
	 * @return A reference to the object that is on the left side of the assignment operator.
	 */
	CircularBuffer<tElement> &operator=(CircularBuffer<tElement> &source)
	{
		if (this!=&source)
		{
			delete []queue;

			head = source.head;
			tail = source.tail;
			length = source.length;
			number_of_entries = source.number_of_entries;

			queue = new tElement[length];

			if (!isEmpty())
				for(unsigned int index=0; index < length; index++)
					queue[index] = source.queue[index];

		}

		return *this;
	}

	/**
	 * @brief Adds an element at the end/tail of the circular buffer.
	 *
	 * @param[in] element a reference to the new element of the queue
	 * @return 0 in case of success, CIRCULAR_BUFFER_FULL error code in case of a full queue
	 */
	BDC_STATE addElement(const tElement &element)
	{
		if (number_of_entries == length)
		{
			queue[tail] = element;
			tail = (tail + 1) % length;
			head = (head + 1) % length;
			return BDC_STATE::DATA_CONTAINER_OVERFLOW;
		}
		else
		{
			queue[tail] = element;
			tail = (tail + 1) % length;
			number_of_entries++;
		}

		return BDC_STATE::DATA_CONTAINER_OPERATION_SUCCESFULL;
	}

	/**
	 * @brief Retrieves an element from the head of the queue.
	 *
	 * @return the value of element from the queue or CIRCULAR_BUFFER_EMPTY error code in case of an empty queue
	 */
	BDC_STATE getElement(tElement &element)
	{
		if (number_of_entries == 0)
		{
			return BDC_STATE::DATA_CONTAINER_EMPTY;
		}
		else
		{
			element = queue[head];
			head = (head + 1) % length;
			number_of_entries--;
		}

		return BDC_STATE::DATA_CONTAINER_OPERATION_SUCCESFULL;
	}

	/**
	 * @brief Checks if the circular buffer is empty or not.
	 *
	 * @return CIRCULAR_BUFFER_EMPTY code if the queue is empty or 0 in case there are elements in the queue
	 */
	bool isEmpty()
	{
		return number_of_entries ? false : true;
	}

    /**
     * @brief Checks if the circular buffer is full
     *
     * @return true if the maximum number of elements is reached, false otherwise
     */
	bool isFull()
	{
		return (number_of_entries == length) ? true : false;
	}

	/**
	 * @brief Clears the content of the circular buffer
     * by making head and tail indexes equal to 0. The memory reserved for the
     * buffer and the actual elements in the buffer are not deleted.
	 */
	void clear()
	{
		tail = head = 0;
		number_of_entries = 0;
	}

    /**
     * @brief Function for getting the actual number of elements currently present in the buffer.
     *
     * @return int number of elements in the buffer
     */
	unsigned int current_number_of_elements() { return number_of_entries; }
    
    /**
     * @brief Provides the amount of elements that can be written into the circular buffer.
     *
     * @return int the size of the internal buffer in nubmer of elements.
     */
	unsigned int size() { return length; }

	const tElement &get_head() const
	{
		return queue[head];
	}

	const tElement &get_tail() const
	{
		return queue[tail];
	}

};

/** @} */

#endif /* CIRCULARBUFFER_H_ */
