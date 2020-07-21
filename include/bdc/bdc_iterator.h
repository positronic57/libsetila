/**
 * @file bdc_iterator.h
 *
 * @brief Template class for implementing iterator for bdc.
 *
 * @author Goce Boshkovski
 * @date 17-Nov-19
 *
 * @copyright GNU General Public License v3
 *
 */

#include <assert.h>

/**
 * \class bdc_iterator
 *
 * @brief Template class for BDC iterator.
 */
template <class tElement> class bdc_iterator
{
public:
	tElement *m_element = nullptr;  /**< Element in the data structure internal buffer pointed by the iterator.*/
    
    /**
     * @brief Default constructor.
     */
	bdc_iterator() = default;

    /**
     * @brief Constructor. Iterator points to a specific element from the data structure
     *
     * @param[in] element address of an element from data structure internal buffer
     */
	bdc_iterator(tElement *element): m_element(element) {};

    /**
     * @brief Impelementation of prefix ++ operator
     * 
     * @return reference for an iterator to the next element in the data stucture buffer
     */
	const bdc_iterator& operator++()	
	{
		m_element++;
		return *this;
	};

    /**
     * @brief Impelentation of the postfix ++ operator
     *
     * @return reference for an iterator to the next element in the data structure buffer
     */
	bdc_iterator operator++(int)
	{
		bdc_iterator temp = *this;
		++(*this);	//Call prefix ++  operator
		return temp;
	}

    /**
     * @brief Implementation of the == operator
     * 
     * @param[in] fqi constant iterator referencing to the left operand of == operator
     * @return ture
     */
	bool operator==(const bdc_iterator& fqi) const
	{
		return (m_element == fqi.m_element) ? true : false;
	}

    /**
     * @brief Impelmentation of != operator
     * 
     * @param[in] fqi constant iterator referencing to the left operand of != operator
     */
	bool operator!=(const bdc_iterator& fqi) const
	{
		return (m_element == fqi.m_element) ? false : true;
	}

    /**
     * @brief Implementation of dereferencing * operator
     *
     * @return a reference to the value pointed by the iterator
     */
	const tElement& operator*() const
	{
		assert(m_element);
		return *m_element;
	}

    /**
     * @brief Implementation of the -> operator.
     *
     * @return an address of the element referenced by the iterator.
     */
	const tElement* operator->() const
	{
		return const_cast<tElement *>(&m_element);
	}
};
