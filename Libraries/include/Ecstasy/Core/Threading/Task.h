#pragma once
#include "move_only_function.h"

namespace Ecstasy::Core::Threading
{
	template <class F>
	using move_only_function_impl = move_only_function<F>;

	using Task = move_only_function_impl<void()>;
}