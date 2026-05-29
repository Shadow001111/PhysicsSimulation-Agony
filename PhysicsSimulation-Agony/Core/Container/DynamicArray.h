#pragma once
#include <cstddef>
#include <cstring>
#include <initializer_list>
#include <iterator>
#include <new>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>

#if false
#include <vector>
template <typename T>
using DynamicArray = std::vector<T>;
#else
template <typename T>
class DynamicArray
{
	T* mData = nullptr;
	size_t mSize = 0;
	size_t mCapacity = 0;

	static constexpr bool can_memcpy_v = std::is_trivially_copyable_v<T>;
	static constexpr bool can_memmove_v = std::is_trivially_copyable_v<T>;

	static void destroy_at(T* pos) noexcept
	{
		if constexpr (!std::is_trivially_destructible_v<T>)
		{
			pos->~T();
		}
	}

	template <typename... Args>
	static void construct_at(T* pos, Args&&... args)
	{
		::new (static_cast<void*>(pos)) T(std::forward<Args>(args)...);
	}

	void destroy_elements() noexcept
	{
		if constexpr (!std::is_trivially_destructible_v<T>)
		{
			for (size_t i = 0; i < mSize; ++i)
			{
				destroy_at(mData + i);
			}
		}
	}

	void deallocate_storage() noexcept
	{
		::operator delete(mData);
		mData = nullptr;
		mCapacity = 0;
	}

	void reallocate(size_t newCapacity)
	{
		if (newCapacity == mCapacity)
			return;

		if (newCapacity == 0)
		{
			destroy_elements();
			deallocate_storage();
			mSize = 0;
			return;
		}

		T* newData = static_cast<T*>(::operator new(newCapacity * sizeof(T)));

		if (mData)
		{
			if constexpr (can_memcpy_v)
			{
				if (mSize != 0)
				{
					std::memcpy(newData, mData, mSize * sizeof(T));
				}
			}
			else
			{
				size_t constructed = 0;
				try
				{
					for (; constructed < mSize; ++constructed)
					{
						construct_at(newData + constructed, std::move_if_noexcept(mData[constructed]));
					}
				}
				catch (...)
				{
					for (size_t i = 0; i < constructed; ++i)
					{
						destroy_at(newData + i);
					}
					::operator delete(newData);
					throw;
				}
			}

			destroy_elements();
			::operator delete(mData);
		}

		mData = newData;
		mCapacity = newCapacity;
	}

	void grow_for_one_more()
	{
		const size_t newCapacity = (mCapacity == 0) ? 1 : (mCapacity * 2);
		reallocate(newCapacity);
	}

	void validate_index(size_t index) const
	{
		if (index >= mSize)
		{
			throw std::out_of_range("Index " + std::to_string(index) +
				" is out of range. DynamicArray size is " + std::to_string(mSize));
		}
	}

	void validate_nonempty(const char* fn) const
	{
		if (mSize == 0)
		{
			throw std::out_of_range(std::string(fn) + " called on empty DynamicArray");
		}
	}

	template <typename ForwardIt>
	void assign_from_forward_range(ForwardIt first, ForwardIt last)
	{
		size_t count = 0;
		for (auto it = first; it != last; ++it)
			++count;

		if (count == 0)
		{
			return;
		}

		mData = static_cast<T*>(::operator new(count * sizeof(T)));
		mCapacity = count;

		size_t constructed = 0;
		try
		{
			for (auto it = first; it != last; ++it, ++constructed)
			{
				construct_at(mData + constructed, *it);
			}
		}
		catch (...)
		{
			for (size_t i = 0; i < constructed; ++i)
			{
				destroy_at(mData + i);
			}
			::operator delete(mData);
			mData = nullptr;
			mCapacity = 0;
			throw;
		}

		mSize = count;
	}

	template <typename... Args>
	void rebuild_with_insertion(size_t index, Args&&... args)
	{
		T* newData = static_cast<T*>(::operator new(mCapacity * sizeof(T)));
		size_t constructed = 0;
		try
		{
			for (size_t i = 0; i < index; ++i, ++constructed)
			{
				construct_at(newData + constructed, std::move_if_noexcept(mData[i]));
			}

			construct_at(newData + constructed, std::forward<Args>(args)...);
			++constructed;

			for (size_t i = index; i < mSize; ++i, ++constructed)
			{
				construct_at(newData + constructed, std::move_if_noexcept(mData[i]));
			}
		}
		catch (...)
		{
			for (size_t i = 0; i < constructed; ++i)
			{
				destroy_at(newData + i);
			}
			::operator delete(newData);
			throw;
		}

		destroy_elements();
		::operator delete(mData);
		mData = newData;
		++mSize;
	}

	void rebuild_without_range(size_t start, size_t end)
	{
		const size_t count = end - start;
		T* newData = static_cast<T*>(::operator new(mCapacity * sizeof(T)));
		size_t constructed = 0;
		try
		{
			for (size_t i = 0; i < start; ++i, ++constructed)
			{
				construct_at(newData + constructed, std::move_if_noexcept(mData[i]));
			}

			for (size_t i = end; i < mSize; ++i, ++constructed)
			{
				construct_at(newData + constructed, std::move_if_noexcept(mData[i]));
			}
		}
		catch (...)
		{
			for (size_t i = 0; i < constructed; ++i)
			{
				destroy_at(newData + i);
			}
			::operator delete(newData);
			throw;
		}

		destroy_elements();
		::operator delete(mData);
		mData = newData;
		mSize -= count;
	}

public:
	using value_type = T;
	using size_type = size_t;
	using iterator = T*;
	using const_iterator = const T*;
	using reverse_iterator = std::reverse_iterator<iterator>;
	using const_reverse_iterator = std::reverse_iterator<const_iterator>;

	// Creation, destruction, copying, moving
	DynamicArray() noexcept = default;

	explicit DynamicArray(size_t size)
	{
		resize(size);
	}

	explicit DynamicArray(size_t size, const T& value)
	{
		resize(size, value);
	}

	explicit DynamicArray(const T* begin, const T* end)
	{
		if (begin == nullptr || end == nullptr)
		{
			throw std::invalid_argument("Null pointer provided to DynamicArray constructor");
		}
		if (end < begin)
		{
			throw std::invalid_argument("Invalid range: end must be greater than or equal to begin");
		}

		const size_t count = static_cast<size_t>(end - begin);
		if (count == 0)
		{
			return;
		}

		mData = static_cast<T*>(::operator new(count * sizeof(T)));
		mCapacity = count;

		if constexpr (can_memcpy_v)
		{
			std::memcpy(mData, begin, count * sizeof(T));
		}
		else
		{
			size_t constructed = 0;
			try
			{
				for (; constructed < count; ++constructed)
				{
					construct_at(mData + constructed, begin[constructed]);
				}
			}
			catch (...)
			{
				for (size_t i = 0; i < constructed; ++i)
				{
					destroy_at(mData + i);
				}
				::operator delete(mData);
				mData = nullptr;
				mCapacity = 0;
				throw;
			}
		}

		mSize = count;
	}

	DynamicArray(std::initializer_list<T> init)
	{
		const size_t count = init.size();
		if (count == 0)
		{
			return;
		}

		mData = static_cast<T*>(::operator new(count * sizeof(T)));
		mCapacity = count;

		if constexpr (can_memcpy_v)
		{
			std::memcpy(mData, init.begin(), count * sizeof(T));
		}
		else
		{
			size_t constructed = 0;
			try
			{
				for (const T& value : init)
				{
					construct_at(mData + constructed, value);
					++constructed;
				}
			}
			catch (...)
			{
				for (size_t i = 0; i < constructed; ++i)
				{
					destroy_at(mData + i);
				}
				::operator delete(mData);
				mData = nullptr;
				mCapacity = 0;
				throw;
			}
		}

		mSize = count;
	}

	template <typename InputIt,
		std::enable_if_t<!std::is_integral_v<InputIt>, int> = 0>
	DynamicArray(InputIt first, InputIt last)
	{
		using category = typename std::iterator_traits<InputIt>::iterator_category;
		static_assert(std::is_base_of_v<std::forward_iterator_tag, category>,
			"DynamicArray range constructor requires at least a forward iterator");

		assign_from_forward_range(first, last);
	}

	~DynamicArray()
	{
		if (mData)
		{
			destroy_elements();
			::operator delete(mData);
		}
	}

	DynamicArray(const DynamicArray& other)
	{
		if (other.mSize == 0)
			return;

		mData = static_cast<T*>(::operator new(other.mSize * sizeof(T)));
		mCapacity = other.mSize;

		if constexpr (can_memcpy_v)
		{
			std::memcpy(mData, other.mData, other.mSize * sizeof(T));
		}
		else
		{
			size_t constructed = 0;
			try
			{
				for (; constructed < other.mSize; ++constructed)
				{
					construct_at(mData + constructed, other.mData[constructed]);
				}
			}
			catch (...)
			{
				for (size_t i = 0; i < constructed; ++i)
				{
					destroy_at(mData + i);
				}
				::operator delete(mData);
				mData = nullptr;
				mCapacity = 0;
				throw;
			}
		}

		mSize = other.mSize;
	}

	DynamicArray& operator=(const DynamicArray& other)
	{
		if (this != &other)
		{
			DynamicArray temp(other);
			swap(temp);
		}
		return *this;
	}

	DynamicArray(DynamicArray&& other) noexcept
		: mData(other.mData), mSize(other.mSize), mCapacity(other.mCapacity)
	{
		other.mData = nullptr;
		other.mSize = 0;
		other.mCapacity = 0;
	}

	DynamicArray& operator=(DynamicArray&& other) noexcept
	{
		if (this != &other)
		{
			if (mData)
			{
				destroy_elements();
				::operator delete(mData);
			}

			mData = other.mData;
			mSize = other.mSize;
			mCapacity = other.mCapacity;

			other.mData = nullptr;
			other.mSize = 0;
			other.mCapacity = 0;
		}
		return *this;
	}

	// Swap
	void swap(DynamicArray& other) noexcept
	{
		using std::swap;
		swap(mData, other.mData);
		swap(mSize, other.mSize);
		swap(mCapacity, other.mCapacity);
	}

	// Element access
	[[nodiscard]] T& operator[](size_t index) noexcept { return mData[index]; }
	[[nodiscard]] const T& operator[](size_t index) const noexcept { return mData[index]; }

	[[nodiscard]] T& at(size_t index)
	{
		validate_index(index);
		return mData[index];
	}

	[[nodiscard]] const T& at(size_t index) const
	{
		validate_index(index);
		return mData[index];
	}

	[[nodiscard]] T& front()
	{
		validate_nonempty("front()");
		return mData[0];
	}

	[[nodiscard]] const T& front() const
	{
		validate_nonempty("front()");
		return mData[0];
	}

	[[nodiscard]] T& back()
	{
		validate_nonempty("back()");
		return mData[mSize - 1];
	}

	[[nodiscard]] const T& back() const
	{
		validate_nonempty("back()");
		return mData[mSize - 1];
	}

	// Modifiers
	void clear() noexcept
	{
		destroy_elements();
		mSize = 0;
	}

	void reserve(size_t newCapacity)
	{
		if (newCapacity > mCapacity)
		{
			reallocate(newCapacity);
		}
	}

	void resize(size_t newSize)
	{
		if (newSize < mSize)
		{
			if constexpr (!std::is_trivially_destructible_v<T>)
			{
				for (size_t i = newSize; i < mSize; ++i)
				{
					destroy_at(mData + i);
				}
			}
			mSize = newSize;
			return;
		}

		if (newSize > mCapacity)
		{
			reallocate(newSize);
		}

		size_t constructed = mSize;
		try
		{
			for (; constructed < newSize; ++constructed)
			{
				construct_at(mData + constructed);
			}
		}
		catch (...)
		{
			for (size_t i = mSize; i < constructed; ++i)
			{
				destroy_at(mData + i);
			}
			throw;
		}

		mSize = newSize;
	}

	void resize(size_t newSize, const T& value)
	{
		if (newSize < mSize)
		{
			if constexpr (!std::is_trivially_destructible_v<T>)
			{
				for (size_t i = newSize; i < mSize; ++i)
				{
					destroy_at(mData + i);
				}
			}
			mSize = newSize;
			return;
		}

		if (newSize > mCapacity)
		{
			reallocate(newSize);
		}

		size_t constructed = mSize;
		try
		{
			for (; constructed < newSize; ++constructed)
			{
				construct_at(mData + constructed, value);
			}
		}
		catch (...)
		{
			for (size_t i = mSize; i < constructed; ++i)
			{
				destroy_at(mData + i);
			}
			throw;
		}

		mSize = newSize;
	}

	void fill(const T& value)
	{
		for (size_t i = 0; i < mSize; ++i)
		{
			mData[i] = value;
		}
	}

	template <typename... Args>
	T& emplace_back(Args&&... args)
	{
		if (mSize >= mCapacity)
		{
			grow_for_one_more();
		}

		construct_at(mData + mSize, std::forward<Args>(args)...);
		++mSize;
		return mData[mSize - 1];
	}

	template <typename... Args>
	T& emplace(size_t index, Args&&... args)
	{
		if (index > mSize)
		{
			throw std::out_of_range("DynamicArray::emplace: position out of range");
		}

		if (mSize >= mCapacity)
		{
			grow_for_one_more();
		}

		if (index == mSize)
		{
			construct_at(mData + mSize, std::forward<Args>(args)...);
			++mSize;
			return mData[mSize - 1];
		}

		if constexpr (std::is_trivially_copyable_v<T>)
		{
			std::memmove(mData + index + 1, mData + index, (mSize - index) * sizeof(T));
			mData[index] = T(std::forward<Args>(args)...);
			++mSize;
			return mData[index];
		}
		else
		{
			rebuild_with_insertion(index, std::forward<Args>(args)...);
			return mData[index];
		}
	}

	template <typename... Args>
	iterator emplace(const_iterator pos, Args&&... args)
	{
		const size_t index = static_cast<size_t>(pos - cbegin());
		emplace(index, std::forward<Args>(args)...);
		return begin() + index;
	}

	void push_back(const T& value)
	{
		emplace_back(value);
	}

	void push_back(T&& value)
	{
		emplace_back(std::move(value));
	}

	void insert(size_t index, const T& value)
	{
		emplace(index, value);
	}

	void insert(size_t index, T&& value)
	{
		emplace(index, std::move(value));
	}

	iterator insert(const_iterator pos, const T& value)
	{
		const size_t index = static_cast<size_t>(pos - cbegin());
		emplace(index, value);
		return begin() + index;
	}

	iterator insert(const_iterator pos, T&& value)
	{
		const size_t index = static_cast<size_t>(pos - cbegin());
		emplace(index, std::move(value));
		return begin() + index;
	}

	void erase(size_t index)
	{
		if (index >= mSize)
		{
			throw std::out_of_range("DynamicArray::erase: position out of range");
		}

		if constexpr (std::is_trivially_copyable_v<T>)
		{
			if (index + 1 < mSize)
			{
				std::memmove(mData + index, mData + index + 1, (mSize - index - 1) * sizeof(T));
			}
			--mSize;
		}
		else
		{
			if (index + 1 == mSize)
			{
				pop_back();
				return;
			}

			rebuild_without_range(index, index + 1);
		}
	}

	void erase(size_t start, size_t end)
	{
		if (start > mSize || end > mSize || start > end)
		{
			throw std::out_of_range("DynamicArray::erase: invalid range");
		}

		const size_t count = end - start;
		if (count == 0)
		{
			return;
		}

		if constexpr (std::is_trivially_copyable_v<T>)
		{
			if (end < mSize)
			{
				std::memmove(mData + start, mData + end, (mSize - end) * sizeof(T));
			}
			mSize -= count;
		}
		else
		{
			rebuild_without_range(start, end);
		}
	}

	iterator erase(const_iterator pos)
	{
		const size_t index = static_cast<size_t>(pos - cbegin());
		erase(index);
		return begin() + index;
	}

	iterator erase(const_iterator first, const_iterator last)
	{
		const size_t start = static_cast<size_t>(first - cbegin());
		const size_t end = static_cast<size_t>(last - cbegin());
		erase(start, end);
		return begin() + start;
	}

	void pop_back()
	{
		validate_nonempty("pop_back()");
		--mSize;
		destroy_at(mData + mSize);
	}

	void shrink_to_fit()
	{
		if (mCapacity > mSize)
		{
			reallocate(mSize);
		}
	}

	// Iterator support
	[[nodiscard]] iterator begin() noexcept { return mData; }
	[[nodiscard]] iterator end() noexcept { return mData ? mData + mSize : mData; }

	[[nodiscard]] const_iterator begin() const noexcept { return mData; }
	[[nodiscard]] const_iterator end() const noexcept { return mData + mSize; }
	[[nodiscard]] const_iterator cbegin() const noexcept { return mData; }
	[[nodiscard]] const_iterator cend() const noexcept { return mData + mSize; }

	[[nodiscard]] reverse_iterator rbegin() noexcept { return reverse_iterator(end()); }
	[[nodiscard]] reverse_iterator rend() noexcept { return reverse_iterator(begin()); }

	[[nodiscard]] const_reverse_iterator rbegin() const noexcept { return const_reverse_iterator(end()); }
	[[nodiscard]] const_reverse_iterator rend() const noexcept { return const_reverse_iterator(begin()); }
	[[nodiscard]] const_reverse_iterator crbegin() const noexcept { return const_reverse_iterator(cend()); }
	[[nodiscard]] const_reverse_iterator crend() const noexcept { return const_reverse_iterator(cbegin()); }

	// Getters
	[[nodiscard]] T* data() noexcept { return mData; }
	[[nodiscard]] const T* data() const noexcept { return mData; }
	[[nodiscard]] size_t size() const noexcept { return mSize; }
	[[nodiscard]] size_t capacity() const noexcept { return mCapacity; }
	[[nodiscard]] bool empty() const noexcept { return mSize == 0; }
};

template <typename T>
void swap(DynamicArray<T>& a, DynamicArray<T>& b) noexcept
{
	a.swap(b);
}
#endif