/**
 * @file stack.h
 * 
 * @brief Template for the stack data container.
 *
 * @date Nov 21, 2019
 * @author Goce Boshkovski
 *
 * @copyright GNU General Public License v3
 */

#ifndef INCLUDE_STACK_H_
#define INCLUDE_STACK_H_

#include "bdc.h"

/** 
 * \defgroup stack_bdc_group Stack
 * \ingroup bdc_group
 * @{
 */

/** 
 *  @brief A template class for a stack.
 *
 *  @tparam tElement single element of the data container
 */
template <class tElement>
class Stack
{
private:
	int top = -1;							/**< Index for the top of the stack. */
	unsigned int length = 32;               /**< Stack size. Default value of 32. */
	tElement *stack = new tElement[32];		/**< The actual buffer.*/

public:
	Stack() = default;
	/**
	 * @brief A constructor.
	 * Sets the size of the stack and alocates a memory for it.
     *
	 * @param[in] len the length of the buffer
	 */
	explicit Stack(unsigned int len): length(len), stack(new tElement[length]) {}

	/**
	 * @brief A virtual destructor of the class.
	 *
	 * It deletes the memory allocated for the stack.
	 */
	virtual ~Stack() { delete [] stack; }

	/**
	 * @brief A copy constructor of the class.
	 *
	 * @param[in] source A source object for the copy operation.
	 */
	Stack(const Stack<tElement> &source)
	{
		top = source.top;
		stack = new tElement[length];

		if (!isEmpty()) {
			for(unsigned int index = 0; index < length; index++)
            {
				stack[index] = source.stack[index];
            }
        }
	}

	/**
	 * @brief Overloading of the assignment operator.
	 *
	 * @param[in] source A source object for the assignment operator.
	 * @return A reference to the object that is on the left side of the assignment operator.
	 */
	Stack<tElement> &operator=(Stack<tElement> &source)
	{
		if (this != &source)
		{
			delete []stack;

			top = source.top;
			length = source.length;

			stack = new tElement[length];

			if (!isEmpty())
				for(unsigned int index=0; index < length; index++)
					stack[index] = source.stack[index];

		}

		return *this;
	}

	/**
	 * @brief Put an element at the top of the stack.
	 *
	 * @param[in] element a reference to the new element
	 * @return DATA_CONTAINER_OPERATION_SUCCESSFULL in case of success, DATA_CONTAINER_FULL code in case of a full queue
	 */
	BDC_STATE push(const tElement &element)
	{
		if ((top + 1) == length)
		{
			return BDC_STATE::DATA_CONTAINER_FULL;
			break;
		}
		else
		{
			top++;
			stack[top] = element;
		}

		return BDC_STATE::DATA_CONTAINER_OPERATION_SUCCESFULL;
	}

	/**
	 * @brief Take the element from the top of the stack.
	 * 
     * @param[out] element from the top of the stack
	 * @return BDC_STATE::DATA_CONTAINER_EMPTY if the stack is empty, BDC_STATE::DATA_CONTAINER_OPERATION_SUCCESFULL if opposite
	 */
	BDC_STATE pop(tElement &element)
	{
		if (top == -1)
		{
			return BDC_STATE::DATA_CONTAINER_EMPTY;
		}
		else
		{
			element = stack[top];
			top--;
		}

		return BDC_STATE::DATA_CONTAINER_OPERATION_SUCCESFULL;
	}

	/**
	 * @brief Checks if the stack is empty
	 *
	 * @return true when the stack is empty, false it there are some elements left
	 */
	bool isEmpty()
	{
		return (top < 0) ? false : true;
	}

    /**
     * @brief Checks if stack is full
     *
     * @return true if there is no space on the stack for new elements, false otherwise
     */
	bool isFull()
	{
		return ((top - 1) == length) ? true : false;
	}
    
    /**
     * @brief Get the number of elements currenty stored in the stack
     *
     * @return number of elements in the stack
     */
	int number_of_elemets()
	{
		return (top + 1);
	}

	/**
	 * @brief Clears the content of the stack by setting the top index to -1.
     * The alocated memory is not released.
	 */
	void clear()
	{
		top = -1;
	}

	/**
	 * @brief Get the index of the top element from the stack.
     *
	 * @return int value of the top index
	 */
	int begin()
	{
		return top;
	}

	/**
	 * @brief Get the index of the bottom of the stack
     *
	 * @return 0
	 */
	int end()
	{
		return 0;
	}
};

/** @} */

#endif /* INCLUDE_STACK_H_ */
