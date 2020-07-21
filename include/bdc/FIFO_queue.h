/**
 * @file FIFO_queue.h
 * 
 * @brief Template class for first in first out type of queue.
 *
 * @author Goce Boshkovski
 * @date 17-Nov-19
 *
 * @copyright GNU General Public License v3
 *
 */

#ifndef FIFIO_QUEUE_H_
#define FIFIO_QUEUE_H_

#include "bdc.h"
#include "bdc_iterator.h"

/** 
 *  \defgroup FIFO_bdc_group FIFO Queue
 *  \ingroup bdc_group
 * @{
 */

/** 
 *  @brief A template class for FIFO queue.
 *
 *  @tparam tElement single element of the data container
 */
template <class tElement>
class FIFO_queue
{
private:
	unsigned int head = 0;					/**< The head of the FIFO queue.*/
	unsigned int tail = 0;					/**< The tail of the FIFO queue. */
	unsigned int length = 32;				/**< Queue size with default value of 32. */
	unsigned int number_of_entries = 0;		/**< Number of entries currently stored in the queue. */
	tElement *queue = new tElement[32];		/**< The actual queue buffer.*/

public:
    /**
     * @brief Definition of a constant_interator data type
     * used as constant iterator.
     */
	using const_iterator = bdc_iterator<tElement>;

public:
    /**
     * @brief Default constructor;
     */
	FIFO_queue() = default;
	
    /**
	 * @brief A constructor.
	 *
	 * The constructor initializes the head and tail values to 0.
	 * It allocates the memory for the queue itself.
	 *
	 * @param[in] len the length of the buffer
	 */
	explicit FIFO_queue(unsigned int len): length(len), queue(new tElement[length]) {}
	
    /**
	 * @brief A virtual destructor of the class.
	 *
	 * It deletes the memory allocated for the buffer.
	 */
	virtual ~FIFO_queue() { delete [] queue; }

	/**
	 * @brief A copy constructor of the class.
	 *
	 * @param[in] source source object for the copy operation.
	 */
	FIFO_queue(const FIFO_queue<tElement> &source)
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
	 * @param[in] source source object for the assignment operator.
	 * @return reference to the object that is on the left side of the assignment operator.
	 */
	FIFO_queue<tElement> &operator=(FIFO_queue<tElement> &source)
	{
		if (this != &source)
		{
			delete []queue;

			head = source.head;
			tail = source.tail;
			length = source.length;
			number_of_entries = source.number_of_entries;

			queue = new tElement[length];

			if (!isEmpty())
            {
				for(unsigned int index=0; index < length; index++)
                {
					queue[index] = source.queue[index];
                }
            }
		}

		return *this;
	}

	/**
	 * @brief Adds an element at the end/tail of the queue.
	 *
	 * @param[in] element a reference to the new element of the queue
	 * @return BDC_STATE::DATA_CONTAINER_OPERATION_SUCCESFULL in case of success, BDC_STATE::DATA_CONTAINER_FULL error code in case of a full queue
	 */
	BDC_STATE addElement(const tElement &element)
	{
		if (number_of_entries == length)
		{
			return BDC_STATE::DATA_CONTAINER_FULL;
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
	 * @return the value of element from the queue or BDC_STATE::DATA_CONTAINER_EMPTY error code in case of an empty queue
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
	 * @brief Checks if the queue is empty.
	 *
	 * @return true if the queue is empty, false if the queue conains elements
	 */
	bool isEmpty()
	{
		return number_of_entries ? false : true;
	}
    
    /**
     * @brief Checks if the queue is full.
     * @return true if the queue is full, false otherwise
     */
	bool isFull()
	{
		return (number_of_entries == length) ? true : false;
	}

	/**
	 * @brief Clears the content of the FIFO queue by setting the values of the head and tail to 0.
     * The buffer itself is not removed.
	 */
	void clear()
	{
		tail = head = 0;
		number_of_entries = 0;
	}

    /**
     * @brief Current number of elements stored in the queue.
     * 
     * @return number of elements stored in the queue
     */
	unsigned int current_number_of_elements() { return number_of_entries; }

    /**
     * @brief The size of the buffer reserved for the queue.
     *
     * @return buffer size
     */
	unsigned int size() { return length; }

	/**
     * @brief Returns an const iterator referring to the firstr element in the FIFO_queue container.
     * For an empty queue, dereferencing the iterator can return undefined values.
     *
     * @return A const iterator to the beginning of the FIFO queue.
     */
    const_iterator begin() const { return bdc_iterator<tElement>(queue); };
	
    /**
     * @brief Returns an const iterator referring to the past-the-last element added in the FIFO_queue container.
     * The past-the-last element is an element that would follow the last element. It does not point to any element.
     * Dereferencing it can lead to undefined behavior if the last element is at the end of the queue.
     *
     *
     * @return A const iterator to the element past the last entry of the FIFO queue
     */
    const_iterator end() const { return bdc_iterator<tElement>(queue + number_of_entries); };
};

/** @} */

#endif /* FIFIO_QUEUE_H_ */
