/**
 * @file: bdc.h
 * 
 * @brief Header file containing the definition of BDC_STATE enum class
 *
 * @author: Goce Boshkovski
 * @date: Nov 30, 2019
 *
 * @copyright GNU General Public License v3
 */

#ifndef BDC_H_
#define BDC_H_

/** \defgroup bdc_group Basic Data Containers */
/* @{ */

/**
 * @brief Enum class that defines the posible outcomes of the operations done
 * on the data containers. 
 */
enum class BDC_STATE: int {
	DATA_CONTAINER_EMPTY = 0,               /**< Data container does not contain elements. */
	DATA_CONTAINER_OPERATION_SUCCESFULL,    /**< Operation is succesfull. */
	DATA_CONTAINER_FULL,                    /**< Data container is full. */
	DATA_CONTAINER_OVERFLOW                 /**< Indicates overflow for certain type of containers when adding a new element. */
};
/* @} */

#endif /* BDC_H_ */
