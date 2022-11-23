#pragma once
#include <boost/math/constants/constants.hpp>

// BOOST_ENABLE_ASSERT_DEBUG_HANDLER is defined for the whole project


constexpr double sqrt2(){
    return boost::math::double_constants::root_two;
}