/*******************************************************************************/
/*                                                                             */
/*  Copyright (c) 2007-2009: Peter Schregle,                                   */
/*  All rights reserved.                                                       */
/*                                                                             */
/*  This file is part of the Fixed Point Math Library.                        */
/*                                                                             */
/*  Redistribution of the Fixed Point Math Library and use in source and      */
/*  binary forms, with or without modification, are permitted provided that    */
/*  the following conditions are met:                                          */
/*  1. Redistributions of source code must retain the above copyright notice,  */
/*     this list of conditions and the following disclaimer.                   */
/*  2. Redistributions in binary form must reproduce the above copyright       */
/*     notice, this list of conditions and the following disclaimer in the     */
/*     documentation and/or other materials provided with the distribution.    */
/*  3. Neither the name of Peter Schregle nor the names of other contributors  */
/*     may be used to endorse or promote products derived from this software   */
/*     without specific prior written permission.                              */
/*                                                                             */
/*  THIS SOFTWARE IS PROVIDED BY PETER SCHREGLE AND CONTRIBUTORS 'AS IS' AND   */
/*  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE      */
/*  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE */
/*  ARE DISCLAIMED. IN NO EVENT SHALL PETER SCHREGLE OR CONTRIBUTORS BE LIABLE */
/*  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL */
/*  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS    */
/*  OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)      */
/*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,        */
/*  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN   */
/*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE            */
/*  POSSIBILITY OF SUCH DAMAGE.                                                */
/*                                                                             */
/*******************************************************************************/


/// \file
/// This file implements a fixed point class, which is hoped to be faster than
/// floating point on some architectures.
//!
//! Paul Dixon helped in making the class compatible with gcc.

#ifndef __fixed_point_h__
#define __fixed_point_h__

#include <limits>
#ifndef _USE_MATH_DEFINES
	#define _USE_MATH_DEFINES
	#define __FPML_DEFINED_USE_MATH_DEFINES__
#endif
#include <cmath>
#include <cstdint>
#include <cerrno>
#include <type_traits>
#include <utility>
#include <ex_type_traits/op_traits.h>

namespace fpml {

/******************************************************************************/
/*                                                                            */
/* fixed_point                                                                */
/*                                                                            */
/******************************************************************************/

/// Multiplication and division of fixed_point numbers need type 
/// promotion. This is default integral promoter for standard integral
/// types.
//!
//! When two 8 bit numbers are multiplied, a 16 bit result is produced.
//! When two 16 bit numbers are multiplied, a 32 bit result is produced.
//! When two 32 bit numbers are multiplied, a 64 bit result is produced.
//! Since the fixed_point class internally relies on integer 
//! multiplication, we need type promotion. After the multiplication we
//! need to adjust the position of the decimal point by shifting the
//! temporary result to the right an appropriate number of bits. 
//! However, if the temporary multiplication result is not kept in a big
//! enough variable, overflow errors will occur and lead to wrong 
//! results. A similar promotion needs to be done to the divisor in the
//! case of division, but here the divisor needs to be shifted to the
//! left an appropriate number of bits.
//!
//! Also this is contained type `limits` for providing numeric limits
//! inteface for promotion type.
template <typename T>
class default_integral_promoter
{
private:

    struct Error_promote_type_not_specialized_for_this_type
    { };

    template<
        /// Pick up unspecialized types.
        typename T,
        /// Make gcc happy.
        typename U=void>
    struct promote_type
	{
		#ifdef _MSC_VER
		typedef Error_promote_type_not_specialized_for_this_type type;
		#endif // #ifdef _MSC_VER
	};

    template<typename U> struct promote_type<int8_t, U> { using type = int16_t; }
    template<typename U> struct promote_type<uint8_t, U> { using type = uint16_t; }
    template<typename U> struct promote_type<int16_t, U> { using type = int32_t; }
    template<typename U> struct promote_type<uint16_t, U> { using type = uint32_t; }
    template<typename U> struct promote_type<int32_t, U> { using type = int64_t; }
    template<typename U> struct promote_type<uint32_t, U> { using type = uint64_t; }

public:
    using type = promote_type<T>::type;
    using limits = std::numeric_limits<type>;
};

// - swapable
// - convert to char, signad char, unsigned char, short,
//   unsigned short, int, unsigned int, long, unsigned long,
//   long long, unsigned long long, bool, float, double,
//   long double
// - is_b_bit_conjunctionable_v<B, B, ???> (for ceil).
// - is_b_modulative_v. (for fmod)
template <typename B, typename TPromotion>
inline constexpr bool is_valid_base_type_v =
    std::is_copy_constructible_v<B> &&
    exttr::op::is_a_rshiftable_v<B, size_t> &&
    exttr::op::is_a_lshiftable_v<B, size_t> &&
    exttr::op::is_b_rshiftable_v<TPromotion, size_t> &&
    exttr::op::is_b_lshiftable_v<TPromotion, size_t> &&
    exttr::op::is_comparable_v<B, B> &&
    exttr::op::is_comparable_v<B, int> &&
    exttr::op::is_negativable_v<B, B> &&
    exttr::op::is_a_summative_v<B, B> &&
    exttr::op::is_a_subtractive_v<B, B> &&
    exttr::op::is_b_multiplicative_v<TPromotion, TPromotion, B> &&
    exttr::op::is_b_dividable_v<TPromotion, TPromotion, B>
    ;

template<
	/// The base type. Must be an integer type. 
	//!
	//! If this is a signed type, the fixed_point number will behave signed too, 
	//! if this is an unsigned type, the fixed_point number will behave 
	//! unsigned.
	typename B,
	/// The integer part bit count.
	uint8_t I,
	/// The fractional part bit count.
	uint8_t F = sizeof (B) * CHAR_BIT - I, // std::numeric_limits<B>::digits - I
    /// Integral type promoter for multiplication and division.
    typename TIntegralPromoter = default_integral_promoter<B>,
    /// Valid numeric limits for base type.
    //!
    //! Numeric limits must realised constants `signed`, `digits` and methods
    //! `max()`, `min()`.
    typename TNumericLimits = std::numeric_limits<B>
>
/// A fixed point type.
//!
//! This type is designed to be a plug-in type to replace the floating point
//! types, such as float, double and long double. While it doesn't offer the
//! precision of these types, its operations are all implemented in integer
//! math, and it is therefore hoped that these operations are faster on non-
//! floating-point enabled hardware.
//!
//! The value uses 0/1 bits for the sign, I bits for the integer part and F bits
//! for the fractional part.
//!
//! Here is an example: a signed 8 bit 5:2 fixed_point type would have the 
//! following layout:
//!
//! fixed_point<signed char, 5, 2>
//!
//!  sign           integer part \ / fractional part
//!	  |                           |
//! +----+----+----+----+----+----+----+----+
//! | S  | I4 | I3 | I2 | I1 | I0 | F0 | F1 |
//! +----+----+----+----+----+----+----+----+
//!
//! where S is the sign-bit, I0 to I4 is the integer part, and F0 to F1 is
//! the fractional part. The range of this type is from -32 to +31.75, the 
//! fractional part can encode multiples of 0.25.
//!
//! The class only implements a few operators directly, the others are generated
//! via the ordered_field_operators, unit_steppable and shiftable templates.
//!
//! The ordered_field_operators template takes and generates (==>) the 
//! following operators:
//! +=   ==>  + (addable),
//! -=   ==>  - (subtractable),
//! *=   ==>  * (multipliable),
//! /=   ==>  / (dividable),
//! <    ==>  > , >=, <= (less_than_comparable),
//! ==   ==>  != (equality_comparable).
//!
//! The unit_steppable template takes and generates (==>) the following 
//! operators:
//! ++   ==>  ++(int), preincrement versus postincrement, (incrementable),
//! --   ==>  --(int), predecrement versus postdecrement, (decrementable).
//!
//! The shiftable template takes and generates the following operators:
//! >>=  ==>  >> (right_shiftable),
//! <<=  ==>  << (left_shiftable).
class fixed_point
{
	// Only integer types qualify for base type. If this line triggers an error,
	// the base type is not an integer type. Note: char does not qualify as an
	// integer because of its uncertainty in definition. Use signed char or
	// unsigned char to be explicit.
    static_assert(is_valid_base_type_v<B, TIntegralPromoter::type>, "Invalid base type.");

	// Make sure that the bit counts are ok. If this line triggers an error, the
	// sum of the bit counts for the fractional and integer parts do not match 
	// the bit count provided by the base type. The sign bit does not count.
    static_assert(I + F == sizeof (B) * CHAR_BIT);

	/// Grant the fixed_point template access to private members. Types with
	/// different template parameters are different types and without this
	/// declaration they do not have access to private members.
	friend class fpml::fixed_point;

	/// Grant the numeric_limits specialization for this fixed_point class 
	/// access to private members.
	friend class std::numeric_limits<fpml::fixed_point<B, I, F> >;

	template<
		/// The power.
		int P,
		/// Make gcc happy.
		typename T = void>
	/// Calculate 2 to the power of F at compile time.
	//!
	//! The fixed_point class needs 2 to the power of P in several locations in
	//! the code. However, the value depends on compile time constants only and
	//! can therefore be calculated at compile time using this template 
	//! trickery. There is no need to call the function pow(2., P) at runtime to
	//! calculate this value.
	//!
	//! The value is calculated by recursively instantiating the power2 template
	//! with successively decrementing P. Finally, 2 to the power of 0 is
	//! terminating the recursion and set to 1.
	struct power2
	{
		static const long long value = 2 * power2<P-1,T>::value;
	};

	template <
		/// Make gcc happy.
		typename P>
	/// Calculate 2 to the power of 0 at compile time.
	//!
	//! The fixed_point class needs 2 to the power of P in several locations in
	//! the code. However, the value depends on compile time constants only and
	//! can therefore be calculated at compile time using this template 
	//! trickery. There is no need to call the function pow(2., P) at runtime to
	//! calculate this value.
	//!
	//! The value is calculated by recursively instantiating the power2 template
	//! with successively decrementing P. Finally, 2 to the power of 0 is
	//! terminating the recursion and set to 1.
	struct power2<0, P>
	{
		static const long long value = 1;
	};

	/// Initializing constructor.
	//!
	//! This constructor takes a value of type B and initializes the internal
	//! representation of fixed_point<B, I, F> with it.
	fixed_point(
		/// The internal representation to use for initialization.
		B value,
		/// This value is not important, it's just here to differentiate from
		/// the other constructors that convert its values.
		bool)
		: value_(value)
	{ }

public:
	/// The base type of this fixed_point class.
	using base_type = B;

    /// Integral type promoter for multiplication and division.
    using integral_promoter = TIntegralPromoter;

    using base_type_limits = TNumericLimits;

	/// The integer part bit count.
	static const unsigned char integer_bit_count = I; 

	/// The fractional part bit count.
	static const unsigned char fractional_bit_count = F;

    static fixed_point<B, I, F> create_with_raw(const base_type & value)
    {
        fixed_point<B, I, F> fp();
        fp._value = value;
        return fp;
    }

	/// Default constructor.
	//!
	//! Just as with built-in types no initialization is done. The value is
	//! undetermined after executing this constructor.
	fixed_point()
        : value_()
	{ }

	template<
		/// The numeric type. Must be integer.
		typename T>
	/// Converting constructor.
	//!
	//! This constructor takes a numeric value of type T and converts it to 
	//! this fixed_point type.
	fixed_point(
		/// The value to convert.
		T value)
		: value_((B)value << F)
	{ 
        // TODO: Need assert.
		//BOOST_CONCEPT_ASSERT((boost::Integer<T>));
	}

	/// Converting constructor.
	//!
	//! This constructor takes a numeric value of type bool and converts it to 
	//! this fixed_point type.
	fixed_point(
		/// The value to convert.
		bool value)
		: value_((B)(value * power2<F>::value))
	{ }

	/// Converting constructor.
	//!
	//! This constructor takes a numeric value of type float and converts it to 
	//! this fixed_point type.
	//!
	//! The conversion is done by multiplication with 2^F and rounding to the 
	//! next integer.
	fixed_point(
		/// The value to convert.
		float value)
		: value_((B)(value * power2<F>::value + (value >= 0 ? .5 : -.5)))
	{ }

	/// Converting constructor.
	//!
	//! This constructor takes a numeric value of type double and converts it to 
	//! this fixed_point type.
	fixed_point(
		/// The value to convert.
		double value)
		: value_((B)(value * power2<F>::value + (value >= 0 ? .5 : -.5)))
	{ }

	/// Converting constructor.
	//!
	//! This constructor takes a numeric value of type long double and converts 
	//! it to this fixed_point type.
	fixed_point(
		/// The value to convert.
		long double value)
		: value_((B)(value * power2<F>::value + (value >= 0 ? .5 : -.5)))
	{ }

	/// Copy constructor.
	fixed_point(
		/// The right hand side.
		fixed_point<B, I, F> const& rhs)
		: value_(rhs.value_)
	{ }

	template<
		/// The other integer part bit count.
		uint8_t I2,
		/// The other fractional part bit count.
		uint8_t F2>
	/// Converting copy constructor.
	fixed_point(
		/// The right hand side.
		fixed_point<B, I2, F2> const& rhs)
		: value_(rhs.value_)
	{ 
		if (I-I2 > 0)
			value_ >>= I-I2;
		if (I2-I > 0)
			value_ <<= I2-I;
	}

	/// Copy assignment operator.
	fpml::fixed_point<B, I, F> & operator =(
		/// The right hand side.
		fpml::fixed_point<B, I, F> const& rhs)
	{
		fpml::fixed_point<B, I, F> temp(rhs);
		swap(temp);
		return *this;
	}

	template<
		/// The other integer part bit count.
		unsigned char I2,
		/// The other fractional part bit count.
		unsigned char F2>
	/// Converting copy assignment operator.
	fpml::fixed_point<B, I, F> & operator =(
		/// The right hand side.
		fpml::fixed_point<B, I2, F2> const& rhs)
	{
		fpml::fixed_point<B, I, F> temp(rhs);
		swap(temp);
		return *this;
	}

	/// Exchanges the elements of two fixed_point objects.
	void swap(
		/// The right hand side.
		fpml::fixed_point<B, I, F> & rhs)
	{
		std::swap(value_, rhs.value_);
	}

	/// Less than operator.
    //!
	//! /return true if less than, false otherwise.
	bool operator <(
		/// Right hand side.
		fpml::fixed_point<B, I, F> const& rhs) const
	{
		return 
			value_ < rhs.value_; 
	}

	/// Equality operator.
    //!
	//! /return true if equal, false otherwise.
	bool operator ==(
		/// Right hand side.
		fpml::fixed_point<B, I, F> const& rhs) const
	{
		return 
			value_ == rhs.value_; 
	}
    /// Other comparison operators.
    #undef FPML_COMP_OP
    #define FPML_COMP_OP(OP) bool operator##OP (fpml::fixed_point<B, I, F> const& rhs) const { return std::rel_ops::operator##OP (*this, rhs); }

    FPML_COMP_OP(!=)
    FPML_COMP_OP(<=)
    FPML_COMP_OP(>)
    FPML_COMP_OP(>=)

    #undef FPML_COMP_OP

	/// Negation operator.
	//!
	//! /return true if equal to zero, false otherwise.
	bool operator !() const
	{
		return value_ == 0; 
	}

	/// Unary minus operator.
	//!
	//! For signed fixed-point types you can apply the unary minus operator to 
	//! get the additive inverse. For unsigned fixed-point types, this operation 
	//! is undefined. Also, shared with the integer base type B, the minimum 
	//! value representable by the type cannot be inverted, since it would yield 
	//! a positive value that is out of range and cannot be represented.
	//!
	//! /return The negative value.
	fpml::fixed_point<B, I, F> operator -() const
	{
		fpml::fixed_point<B, I, F> result;
		result.value_ = -value_;
		return result;
	}

	/// Increment.
	//!
	//! /return A reference to this object.
	fpml::fixed_point<B, I, F> & operator ++()
	{
		value_ += power2<F>::value;
		return *this;
	}

    fpml::fixed_point<B, I, F> & operator ++(int)
	{
        fpml::fixed_point<B, I, F> old;
        ++(*this);
        return old;
	}

	/// Decrement.
	//!
	//! /return A reference to this object.
	fpml::fixed_point<B, I, F> & operator --()
	{
		value_ -= power2<F>::value;
		return *this;
	}

    fpml::fixed_point<B, I, F> & operator --(int)
	{
		fpml::fixed_point<B, I, F> old;
        --(*this);
        return old;
	}

	/// Addition.
	//!
	//! /return A reference to this object.
	fpml::fixed_point<B, I, F> & operator +=(
		/// Summand for addition.
		fpml::fixed_point<B, I, F> const& summand)
	{
		value_ += summand.value_;
		return *this;
	}

    fpml::fixed_point<B, I, F> operator +(
        /// Summand for addition.
        fpml::fixed_point<B, I, F> const& summand)
    {
        fpml::fixed_point<B, I, F> result(*this);
        result += summand;
        return result;
    }

	/// Subtraction.
	//!
	//! /return A reference to this object.
	fpml::fixed_point<B, I, F> & operator -=(
		/// Diminuend for subtraction.
		fpml::fixed_point<B, I, F> const& diminuend)
	{
		value_ -= diminuend.value_;
		return *this;
	}

    fpml::fixed_point<B, I, F> operator -(
        /// Diminuend for subtraction.
        fpml::fixed_point<B, I, F> const& diminuend)
    {
        fpml::fixed_point<B, I, F> result(*this);
        result -= diminuend;
        return result;
    }

	/// Multiplication.
	//!
	//! /return A reference to this object.
	fpml::fixed_point<B, I, F> & operator *=(
		/// Factor for mutliplication.
		fpml::fixed_point<B, I, F> const& factor)
	{
		value_ = (static_cast<integral_promoter::type>
			(value_) * factor.value_) >> F;
		return *this;
	}

    fpml::fixed_point<B, I, F> operator *(
        /// Factor for mutliplication.
        fpml::fixed_point<B, I, F> const& factor)
    {
        fpml::fixed_point<B, I, F> result(*this);
        result *= factor;
        return result;
    }

	/// Division.
	//!
	//! /return A reference to this object.
	fpml::fixed_point<B, I, F> & operator /=(
		/// Divisor for division.
		fpml::fixed_point<B, I, F> const& divisor)
	{
		value_ = (static_cast<integral_promoter::type>
			(value_) << F) / divisor.value_;
		return *this;
	}

    fpml::fixed_point<B, I, F> operator /(
        /// Factor for division.
        fpml::fixed_point<B, I, F> const& divisor)
    {
        fpml::fixed_point<B, I, F> result(*this);
        result /= divisor;
        return result;
    }

	/// Shift right.
	//!
	//! /return A reference to this object.
	fpml::fixed_point<B, I, F> & operator >>=(
		/// Count of positions to shift.
		size_t shift)
	{
		value_ >>= shift;
		return *this;
	}

    fpml::fixed_point<B, I, F> operator >>(
        /// Count of positions to shift.
        size_t shift)
    {
        fpml::fixed_point<B, I, F> result(*this);
        result >>= shift;
        return result;
    }

	/// Shift left.
	//!
	//! /return A reference to this object.
	fpml::fixed_point<B, I, F> & operator <<=(
		/// Count of positions to shift.
		size_t shift)
	{
		value_ <<= shift;
		return *this;
	}

    fpml::fixed_point<B, I, F> operator <<(
        /// Count of positions to shift.
        size_t shift)
    {
        fpml::fixed_point<B, I, F> result(*this);
        result <<= shift;
        return result;
    }

	/// Convert to char.
	//!
	//! /return The value converted to char.
	operator char() const
	{
		return static_cast<char>(value_ >> F);	
	}

	/// Convert to signed char.
	//!
	//! /return The value converted to signed char.
	operator signed char() const
	{
		return static_cast<signed char>(value_ >> F);	
	}

	/// Convert to unsigned char.
	//!
	//! /return The value converted to unsigned char.
	operator unsigned char() const
	{
		return static_cast<unsigned char>(value_ >> F);	
	}

	/// Convert to short.
	//!
	//! /return The value converted to short.
	operator short() const
	{
		return static_cast<short>(value_ >> F);	
	}

	/// Convert to unsigned short.
	//!
	//! /return The value converted to unsigned short.
	operator unsigned short() const
	{
		return static_cast<unsigned short>(value_ >> F);	
	}

	/// Convert to int.
	//!
	//! /return The value converted to int.
	operator int() const
	{
		return static_cast<int>(value_ >> F);	
	}

	/// Convert to unsigned int.
	//!
	//! /return The value converted to unsigned int.
	operator unsigned int() const
	{
		return static_cast<unsigned int>(value_ >> F);	
	}

	/// Convert to long.
	//!
	//! /return The value converted to long.
	operator long() const
	{
		return static_cast<long>(value_ >> F);	
	}

	/// Convert to unsigned long.
	//!
	//! /return The value converted to unsigned long.
	operator unsigned long() const
	{
		return static_cast<unsigned long>(value_ >> F);	
	}

	/// Convert to long long.
	//!
	//! /return The value converted to long long.
	operator long long() const
	{
		return static_cast<long long>(value_ >> F);	
	}

	/// Convert to unsigned long long.
	//!
	//! /return The value converted to unsigned long long.
	operator unsigned long long() const
	{
		return static_cast<unsigned long long>(value_ >> F);	
	}

	/// Convert to a bool.
	//!
	//! /return The value converted to a bool.
	operator bool() const
	{
		return static_cast<bool>value_;	
	}

    /// Convert to a base type.
	//!
    //! Such as base type could be one of previously used
    //! types for casting we can not be use cast operator
    //! for this.
    //!
	//! /return The value converted to a base type.

    base_type cast_to_base() const
    {
        return value_ >> F;
    }

    /// Give access to base type.
    //!
    //! /return The reference to base type value.
    base_type & data()
    {
        return value_;
    }

	/// Convert to a float.
	//!
	//! /return The value converted to a float.
	operator float() const
	{
        // TODO: Needs improvement: composite integer cannot be assigned to float.
		return (float)value_ / power2<F>::value;	
	}

	/// Convert to a double.
	//!
	//! /return The value converted to a double.
	operator double() const
	{
        // TODO: Needs improvement: composite integer cannot be assigned to double.
		return (double)value_ / power2<F>::value;	
	}

	/// Convert to a long double.
	//!
	//! /return The value converted to a long double.
	operator long double() const
	{
        // TODO: Needs improvement: composite integer cannot be assigned to long double.
		return (long double)value_ / power2<F>::value;	
	}

	/**************************************************************************/
	/*                                                                        */
	/* fabs                                                                   */
	/*                                                                        */
	/**************************************************************************/

	/// Calculates the absolute value.
	//! 
	//! The fabs function computes the absolute value of its argument.
	//!
	//! /return The absolute value of the argument.
	friend fpml::fixed_point<B, I, F> fabs(
		/// The argument to the function.
		fpml::fixed_point<B, I, F> x)
	{
        // TODO: Check sign bit.
		return x < fpml::fixed_point<B, I, F>(0) ? -x : x;
	}

	/**************************************************************************/
	/*                                                                        */
	/* ceil                                                                   */
	/*                                                                        */
	/**************************************************************************/

	/// Calculates the ceiling value.
	//! 
	//! The ceil function computes the smallest integral value not less than 
	//! its argument.
	//!
	//! /return The smallest integral value not less than the argument.
	friend fpml::fixed_point<B, I, F> ceil(
		/// The argument to the function.
		fpml::fixed_point<B, I, F> x)
	{
		fpml::fixed_point<B, I, F> result;
        // TODO: Change to bit operations:
        // x.value_ & ~(power2<F>::value-1):
        // int16_t(power2<4>::value-1) is a 0b00000000'00001111
		result.value_ = x.value_ & ~(power2<F>::value-1);
		return result + fpml::fixed_point<B, I, F>(
			x.value_ & (power2<F>::value-1) ? 1 : 0);
	}

	/**************************************************************************/
	/*                                                                        */
	/* floor                                                                  */
	/*                                                                        */
	/**************************************************************************/

	/// Calculates the floor.
	//! 
	//! The floor function computes the largest integral value not greater than 
	//! its argument.
	//!
	//! /return The largest integral value not greater than the argument.
	friend fpml::fixed_point<B, I, F> floor(
		/// The argument to the function.
		fpml::fixed_point<B, I, F> x)
	{
        // TODO: Change to bit operations:
        // x.value_ & ~(power2<F>::value-1):
        // int16_t(power2<4>::value-1) is a 0b00000000'00001111
		fpml::fixed_point<B, I, F> result;
		result.value_ = x.value_ & ~(power2<F>::value-1);
		return result;
	}

	/**************************************************************************/
	/*                                                                        */
	/* fmod                                                                   */
	/*                                                                        */
	/**************************************************************************/

	/// Calculates the remainder.
	//! 
	//! The fmod function computes the fixed point remainder of x/y.
	//!
	//! /return The fixed point remainder of x/y.
	friend fpml::fixed_point<B, I, F> fmod(
		/// The argument to the function.
		fpml::fixed_point<B, I, F> x,
		/// The argument to the function.
		fpml::fixed_point<B, I, F> y)
	{
		fpml::fixed_point<B, I, F> result;
		result.value_ = x.value_ % y.value_;
		return result;
	}

	/**************************************************************************/
	/*                                                                        */
	/* modf                                                                   */
	/*                                                                        */
	/**************************************************************************/

	/// Split in integer and fraction parts.
	//! 
	//! The modf function breaks the argument into integer and fraction parts,
	//! each of which has the same sign as the argument. It stores the integer
	//! part in the object pointed to by ptr.
	//!
	//! /return The signed fractional part of x/y.
	friend fpml::fixed_point<B, I, F> modf(
		/// The argument to the function.
		fpml::fixed_point<B, I, F> x,
		/// The pointer to the integer part.
		fpml::fixed_point<B, I, F> * ptr)
	{
		fpml::fixed_point<B, I, F> integer;
		integer.value_ = x.value_ & ~(power2<F>::value-1);
		*ptr = x < fpml::fixed_point<B, I, F>(0) ? 
			integer + fpml::fixed_point<B, I, F>(1) : integer;

		fpml::fixed_point<B, I, F> fraction;
		fraction.value_ = x.value_ & (power2<F>::value-1);

		return x < fpml::fixed_point<B, I, F>(0) ? -fraction : fraction;
	}

	/**************************************************************************/
	/*                                                                        */
	/* exp                                                                    */
	/*                                                                        */
	/**************************************************************************/

	/// Calculates the exponential.
	//! 
	//! The function computes the exponential function of x. The algorithm uses
	//! the identity e^(a+b) = e^a * e^b.
	//!
	//! /return The exponential of the argument.
	friend fpml::fixed_point<B, I, F> exp(
		/// The argument to the exp function.
		fpml::fixed_point<B, I, F> x)
	{
		// a[x] = exp( (1/2) ^ x ), x: [0 ... 31]
		fpml::fixed_point<B, I, F> a[] = {
			1.64872127070012814684865078781, 
			1.28402541668774148407342056806, 
			1.13314845306682631682900722781, 
			1.06449445891785942956339059464, 
			1.03174340749910267093874781528, 
			1.01574770858668574745853507208, 
			1.00784309720644797769345355976, 
			1.00391388933834757344360960390, 
			1.00195503359100281204651889805, 
			1.00097703949241653524284529261, 
			1.00048840047869447312617362381, 
			1.00024417042974785493700523392, 
			1.00012207776338377107650351967, 
			1.00006103701893304542177912060, 
			1.00003051804379102429545128481, 
			1.00001525890547841394814004262, 
			1.00000762942363515447174318433, 
			1.00000381470454159186605078771, 
			1.00000190735045180306002872525, 
			1.00000095367477115374544678825, 
			1.00000047683727188998079165439, 
			1.00000023841860752327418915867, 
			1.00000011920929665620888994533, 
			1.00000005960464655174749969329, 
			1.00000002980232283178452676169,
			1.00000001490116130486995926397,
			1.00000000745058062467940380956,
			1.00000000372529030540080797502,
			1.00000000186264515096568050830,
			1.00000000093132257504915938475,
			1.00000000046566128741615947508 };

		fpml::fixed_point<B, I, F> e(2.718281828459045);

		fpml::fixed_point<B, I, F> y(1);
		for (int i=F-1; i>=0; --i)
		{
			if (!(x.value_ & 1<<i))
				y *= a[F-i-1];
		}

		int x_int = (int)(floor(x));
		if (x_int<0)
		{
			for (int i=1; i<=-x_int; ++i)
				y /= e;
		}
		else
		{
			for (int i=1; i<=x_int; ++i)
				y *= e;
		}

		return y;
	}

	/**************************************************************************/
	/*                                                                        */
	/* cos                                                                    */
	/*                                                                        */
	/**************************************************************************/

	/// Calculates the cosine.
	//! 
	//! The algorithm uses a MacLaurin series expansion.
	//!
	//! First the argument is reduced to be within the range -Pi .. +Pi. Then
	//! the MacLaurin series is expanded. The argument reduction is problematic 
	//! since Pi cannot be represented exactly. The more rounds are reduced the
	//! less significant is the argument (every reduction round makes a slight
	//! error), to the extent that the reduced argument and consequently the 
	//! result are meaningless.
	//!
	//! The argument reduction uses one division. The series expansion uses 3
	//! additions and 4 multiplications.
	//!
	//! /return The cosine of the argument.
	friend fpml::fixed_point<B, I, F> cos(
		/// The argument to the cos function.
		fpml::fixed_point<B, I, F> x)
	{
		fpml::fixed_point<B, I, F> x_ = 
			fmod(x, fpml::fixed_point<B, I, F>(M_PI * 2));
		if (x_ > fpml::fixed_point<B, I, F>(M_PI))
			x_ -= fpml::fixed_point<B, I, F>(M_PI * 2);

		fpml::fixed_point<B, I, F> xx = x_ * x_;

		fpml::fixed_point<B, I, F> y = - xx * 
			fpml::fixed_point<B, I, F>(1. / (2 * 3 * 4 * 5 * 6));
		y += fpml::fixed_point<B, I, F>(1. / (2 * 3 * 4));
		y *= xx;
		y -= fpml::fixed_point<B, I, F>(1. / (2));
		y *= xx;
		y += fpml::fixed_point<B, I, F>(1);

		return y;
	}

	/**************************************************************************/
	/*                                                                        */
	/* sin                                                                    */
	/*                                                                        */
	/**************************************************************************/

	/// Calculates the sine.
	//! 
	//! The algorithm uses a MacLaurin series expansion.
	//!
	//! First the argument is reduced to be within the range -Pi .. +Pi. Then
	//! the MacLaurin series is expanded. The argument reduction is problematic 
	//! since Pi cannot be represented exactly. The more rounds are reduced the
	//! less significant is the argument (every reduction round makes a slight
	//! error), to the extent that the reduced argument and consequently the 
	//! result are meaningless.
	//!
	//! The argument reduction uses one division. The series expansion uses 3
	//! additions and 5 multiplications.
	//!
	//! /return The sine of the argument.
	friend fpml::fixed_point<B, I, F> sin(
		/// The argument to the sin function.
		fpml::fixed_point<B, I, F> x)
	{
		fpml::fixed_point<B, I, F> x_ = 
			fmod(x, fpml::fixed_point<B, I, F>(M_PI * 2));
		if (x_ > fpml::fixed_point<B, I, F>(M_PI))
			x_ -= fpml::fixed_point<B, I, F>(M_PI * 2);

		fpml::fixed_point<B, I, F> xx = x_ * x_;

		fpml::fixed_point<B, I, F> y = - xx * 
			fpml::fixed_point<B, I, F>(1. / (2 * 3 * 4 * 5 * 6 * 7));
		y += fpml::fixed_point<B, I, F>(1. / (2 * 3 * 4 * 5));
		y *= xx;
		y -= fpml::fixed_point<B, I, F>(1. / (2 * 3));
		y *= xx;
		y += fpml::fixed_point<B, I, F>(1);
		y *= x_;

		return y;
	}

	/**************************************************************************/
	/*                                                                        */
	/* sqrt                                                                   */
	/*                                                                        */
	/**************************************************************************/

	/// Calculates the square root.
	//! 
	//! The sqrt function computes the nonnegative square root of its argument.
	//! A domain error results if the argument is negative.
	//!
	//! Calculates an approximation of the square root using an integer 
	//! algorithm. The algorithm is described in Wikipedia: 
	//! http://en.wikipedia.org/wiki/Methods_of_computing_square_roots
	//!
	//! The algorithm seems to have originated in a book on programming abaci by 
	//! Mr C. Woo.
	//!
	//! /return The square root of the argument. If the argument is negative, 
	//! the function returns 0.
	friend fpml::fixed_point<B, I, F> sqrt(
		/// The argument to the square root function, a nonnegative fixed-point
		/// value.
		fpml::fixed_point<B, I, F> x)
	{
		if (x < fpml::fixed_point<B, I, F>(0))
		{
			errno = EDOM;
			return 0;
		}

		typename fpml::fixed_point<B, I, F>::template promote_type<B>::type op = 
			static_cast<typename fpml::fixed_point<B, I, F>::template promote_type<B>::type>(
				x.value_) << (I - 1);
		typename fpml::fixed_point<B, I, F>::template promote_type<B>::type res = 0;
		typename fpml::fixed_point<B, I, F>::template promote_type<B>::type one = 
			(typename fpml::fixed_point<B, I, F>::template promote_type<B>::type)1 << 
				(integral_promoter::limits::digits - 1); 

		while (one > op)
			one >>= 2;

		while (one != 0)
		{
			if (op >= res + one)
			{
				op = op - (res + one);
				res = res + (one << 1);
			}
			res >>= 1;
			one >>= 2;
		}

		fpml::fixed_point<B, I, F> root;
		root.value_ = static_cast<B>(res);
		return root;
	}

private:
	/// The value in fixed point format.
	B value_;
};


template<
	/// The input stream type.
	typename S,
	/// The base type of the fixed_point type. 
	typename B,
	/// The integer part bit count of the fixed_point type.
	unsigned char I,
	/// The fractional part bit count of the fixed_point type.
	unsigned char F>
/// Stream input operator.
//!
//! A value is first input to type double and then the read value is converted
//! to type fixed_point before it is returned.
//!
//! /return A reference to this input stream.
S & operator>>(
	/// The input stream.
	S & s, 
	/// A reference to the value to be read.
	fpml::fixed_point<B, I, F> & v)
{
    // TODO: Edit it.
	double value=0.;
	s >> value;
	if (s)
		v = value;
	return s;
}


template<
	/// The output stream type.
	typename S,
	/// The base type of the fixed_point type. 
	typename B,
	/// The integer part bit count of the fixed_point type.
	unsigned char I,
	/// The fractional part bit count of the fixed_point type.
	unsigned char F>
/// Stream output operator.
//!
//! The fixed_point value is first converted to type double and then the output
//! operator for type double is called.
//!
//! /return A reference to this output stream.
S & operator<<(
	/// The output stream.
	S & s, 
	/// A const reference to the value to be written.
	fpml::fixed_point<B, I, F> const& v)
{
    // TODO: Edit it.
	double value = v;
	s << value;
	return s;
}


} // namespace fmpl


namespace std
{

/******************************************************************************/
/*                                                                            */
/* numeric_limits<fpml::fixed_point<B, I, F> >                                */
/*                                                                            */
/******************************************************************************/

template<
	/// The base type of the fixed_point type. 
	typename B,
	/// The integer part bit count of the fixed_point type.
	unsigned char I,
	/// The fractional part bit count of the fixed_point type.
	unsigned char F>
class numeric_limits<fpml::fixed_point<B, I, F> >
{
public:
	/// The fixed_point type. This numeric_limits specialization is specialized
	/// for this type.
	typedef fpml::fixed_point<B, I, F> fp_type;

	/// Tests whether a type allows denormalized values.
	//!
	//! An enumeration value of type const float_denorm_style, indicating 
	//! whether the type allows denormalized values. The fixed_point class does
	//! not have denormalized values.
	//!
	//! The member is always set to denorm_absent.
	static const float_denorm_style has_denorm = denorm_absent;

	/// Tests whether loss of accuracy is detected as a denormalization loss 
	/// rather than as an inexact result.
	//!
	//! The fixed_point class does not have denormalized values.
	//!
	//! The member is always set to false.
	static const bool has_denorm_loss = false;

	/// Tests whether a type has a representation for positive infinity.
	//!
	//! The fixed_point class does not have a representation for positive
	//! infinity.
	//!
	//! The member is always set to false.
	static const bool has_infinity = false;

	/// Tests whether a type has a representation for a quiet not a number 
	/// (NAN), which is nonsignaling.
	//!
	//! The fixed_point class does not have a quiet NAN.
	//!
	//! The member is always set to false.
	static const bool has_quiet_NaN = false;

	/// Tests whether a type has a representation for signaling not a number 
	//! (NAN).
	//!
	//! The fixed_point class does not have a signaling NAN.
	//!
	//! The member is always set to false.
	static const bool has_signaling_NaN = false;

	/// Tests if the set of values that a type may represent is finite.
	//!
	//! The fixed_point type has a bounded set of representable values.
	//!
	//! The member is always set to true.
	static const bool is_bounded = true;

	/// Tests if the calculations done on a type are free of rounding errors.
	//!
	//! The fixed_point type is considered exact.
	//!
	//! The member is always set to true.
	static const bool is_exact = true;

	/// Tests if a type conforms to IEC 559 standards.
	//!
	//! The fixed_point type does not conform to IEC 559 standards.
	//!
	//! The member is always set to false.
	static const bool is_iec559 = false;

	/// Tests if a type has an integer representation.
	//!
	//! The fixed_point type behaves like a real number and thus has not
	//! integer representation.
	//!
	//! The member is always set to false.
	static const bool is_integer = false;

	/// Tests if a type has a modulo representation.
	//!
	//! A modulo representation is a representation where all results are 
	//! reduced modulo some value. The fixed_point class does not have a
	//! modulo representation.
	//!
	//! The member is always set to false.
	static const bool is_modulo = false;

	/// Tests if a type has a signed representation.
	//!
	//! The member stores true for a type that has a signed representation, 
	//! which is the case for all fixed_point types with a signed base type.
	//! Otherwise it stores false.
	static const bool is_signed = fp_type::base_type_limits::is_signed;

	/// Tests if a type has an explicit specialization defined in the template 
	/// class numeric_limits.
	//!
	//! The fixed_point class has an explicit specialization.
	//!
	//! The member is always set to true.
	static const bool is_specialized = true;

	/// Tests whether a type can determine that a value is too small to 
	/// represent as a normalized value before rounding it.
	//!
	//! Types that can detect tinyness were included as an option with IEC 559 
	//! floating-point representations and its implementation can affect some 
	//! results.
	//!
	//! The member is always set to false.
	static const bool tinyness_before = false;

	/// Tests whether trapping that reports on arithmetic exceptions is 
	//! implemented for a type.
	//!
	//! The member is always set to false.
	static const bool traps = false;

	/// Returns a value that describes the various methods that an 
	/// implementation can choose for rounding a real value to an integer 
	/// value.
	//!
	//! The member is always set to round_toward_zero.
	static const float_round_style round_style = round_toward_zero;

	/// Returns the number of radix digits that the type can represent without 
	/// loss of precision.
	//!
	//! The member stores the number of radix digits that the type can represent 
	//! without change.
	//!
	//! The member is set to the template parameter I (number of integer bits).
	static const int digits = I;

	/// Returns the number of decimal digits that the type can represent without 
	/// loss of precision.
	//!
	//! The member is set to the number of decimal digits that the type can 
	//! represent.
	static const int digits10 = (int)(digits * 301. / 1000. + .5);

	static const int max_exponent = 0;
	static const int max_exponent10 = 0;
	static const int min_exponent = 0;
	static const int min_exponent10 = 0;
	static const int radix = 0;

	/// The minimum value of this type.
	//!
	//! /return The minimum value representable with this type.
	static fp_type (min)()
	{
		fp_type minimum;
		minimum.value_ = (fp_type::base_type_limits::min)();
		return minimum;
	}

	/// The maximum value of this type.
	//!
	//! /return The maximum value representable with this type.
	static fp_type (max)()
	{
		fp_type maximum;
		maximum.value_ = (fp_type::base_type_limits::max)();
		return maximum;
	}

	/// The function returns the difference between 1 and the smallest value 
	/// greater than 1 that is representable for the data type.
	//!
	//! /return The smallest effective increment from 1.0.
	static fp_type epsilon()
	{
		fp_type epsilon;
		epsilon.value_ = 1;
		return epsilon;
	}

	/// Returns the maximum rounding error for the type.
	//!
	//! The maximum rounding error for the fixed_point type is 0.5.
	//!
	//! /return Always returns 0.5.
	static fp_type round_error()
	{
		return (fp_type)(0.5);
	}

	static fp_type denorm_min()
	{
		return (fp_type)(0);
	}

	static fp_type infinity()
	{
		return (fp_type)(0);
	}

	static fp_type quiet_NaN()
	{
		return (fp_type)(0);
	}

	static fp_type signaling_NaN()
	{
		return (fp_type)(0);
	}
};


} // namespace std

#ifdef __FPML_DEFINED_USE_MATH_DEFINES__
	#undef _USE_MATH_DEFINES
	#undef __FPML_DEFINED_USE_MATH_DEFINES__
#endif

#endif // __fixed_point_h__
