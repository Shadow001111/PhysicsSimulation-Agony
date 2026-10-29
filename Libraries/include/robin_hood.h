#pragma once
#include <algorithm>
#include <bit>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <initializer_list>
#include <iterator>
#include <limits>
#include <memory>
#include <new>
#include <stdexcept>
#include <string>
#include <string_view>
#include <tuple>
#include <type_traits>
#include <utility>
#ifdef _MSC_VER
#define ROBIN_HOOD_NOINLINE __declspec(noinline)
#else
#define ROBIN_HOOD_NOINLINE __attribute__((noinline))
#endif
namespace robin_hood
{
namespace detail
{
template <typename T>
[[nodiscard]] inline T unalignedLoad(void const* ptr) noexcept
{
    T t;
    std::memcpy(&t, ptr, sizeof(T));
    return t;
}
template <typename To, typename From>
[[nodiscard]] inline To* castPtr(From* ptr) noexcept
{
    return static_cast<To*>(static_cast<void*>(ptr));
}
template <typename E, typename... Args>
[[noreturn]] ROBIN_HOOD_NOINLINE void doThrow(Args&&... args)
{
#if defined(__cpp_exceptions) || defined(_CPPUNWIND)
    throw E(std::forward<Args>(args)...);
#else
    ((void)args, ...);
    std::abort();
#endif
}
template <typename T>
[[nodiscard]] inline void* allocateBytes(std::size_t numBytes)
{
    if constexpr (alignof(T) > __STDCPP_DEFAULT_NEW_ALIGNMENT__)
    {
        return ::operator new(numBytes, std::align_val_t{ alignof(T) });
    }
    else
    {
        return ::operator new(numBytes);
    }
}
template <typename T>
inline void freeBytes(void* ptr) noexcept
{
    if constexpr (alignof(T) > __STDCPP_DEFAULT_NEW_ALIGNMENT__)
    {
        ::operator delete(ptr, std::align_val_t{ alignof(T) });
    }
    else
    {
        ::operator delete(ptr);
    }
}
}
struct is_transparent_tag
{
};
// std::pair is not trivially copyable, which would rule out memcpy-cloning of the table.
template <typename T1, typename T2>
struct pair
{
    using first_type = T1;
    using second_type = T2;
    constexpr pair() noexcept(std::is_nothrow_default_constructible_v<T1> && std::is_nothrow_default_constructible_v<T2>)
        requires(std::is_default_constructible_v<T1> && std::is_default_constructible_v<T2>) :
        first(),
        second()
    {
    }
    explicit constexpr pair(std::pair<T1, T2> const& o) noexcept(std::is_nothrow_copy_constructible_v<T1> && std::is_nothrow_copy_constructible_v<T2>) :
        first(o.first),
        second(o.second)
    {
    }
    explicit constexpr pair(std::pair<T1, T2>&& o) noexcept(std::is_nothrow_move_constructible_v<T1> && std::is_nothrow_move_constructible_v<T2>) :
        first(std::move(o.first)),
        second(std::move(o.second))
    {
    }
    constexpr pair(T1&& a, T2&& b) noexcept(std::is_nothrow_move_constructible_v<T1> && std::is_nothrow_move_constructible_v<T2>) :
        first(std::move(a)),
        second(std::move(b))
    {
    }
    template <typename U1, typename U2>
    constexpr pair(U1&& a, U2&& b) noexcept(std::is_nothrow_constructible_v<T1, U1&&> && std::is_nothrow_constructible_v<T2, U2&&>) :
        first(std::forward<U1>(a)),
        second(std::forward<U2>(b))
    {
    }
    template <typename... U1, typename... U2>
    constexpr pair(std::piecewise_construct_t, std::tuple<U1...> a, std::tuple<U2...> b) noexcept(std::is_nothrow_constructible_v<T1, U1&&...> && std::is_nothrow_constructible_v<T2, U2&&...>) :
        pair(a, b, std::index_sequence_for<U1...>{}, std::index_sequence_for<U2...>{})
    {
    }
    template <typename... U1, std::size_t... I1, typename... U2, std::size_t... I2>
    constexpr pair(std::tuple<U1...>& a, std::tuple<U2...>& b, std::index_sequence<I1...>, std::index_sequence<I2...>) noexcept(std::is_nothrow_constructible_v<T1, U1&&...> && std::is_nothrow_constructible_v<T2, U2&&...>) :
        first(std::forward<U1>(std::get<I1>(a))...),
        second(std::forward<U2>(std::get<I2>(b))...)
    {
        (void)a;
        (void)b;
    }
    void swap(pair& o) noexcept(std::is_nothrow_swappable_v<T1> && std::is_nothrow_swappable_v<T2>)
    {
        using std::swap;
        swap(first, o.first);
        swap(second, o.second);
    }
    friend void swap(pair& a, pair& b) noexcept(std::is_nothrow_swappable_v<T1> && std::is_nothrow_swappable_v<T2>)
    {
        a.swap(b);
    }
    friend bool operator==(pair const&, pair const&) = default;
    friend auto operator<=>(pair const&, pair const&) = default;
    T1 first;
    T2 second;
};
namespace detail
{
template <typename T>
inline constexpr bool isPair = false;
template <typename A, typename B>
inline constexpr bool isPair<pair<A, B>> = true;
}
template <std::size_t I, typename P>
    requires(I < 2 && detail::isPair<std::remove_cvref_t<P>>)
constexpr decltype(auto) get(P&& p) noexcept
{
    if constexpr (I == 0)
    {
        return (std::forward<P>(p).first);
    }
    else
    {
        return (std::forward<P>(p).second);
    }
}
inline std::size_t hash_bytes(void const* ptr, std::size_t len) noexcept
{
    constexpr std::uint64_t m = 0xc6a4a7935bd1e995ULL;
    constexpr std::uint64_t seed = 0xe17a1465ULL;
    constexpr unsigned r = 47;
    auto const* const data = static_cast<std::uint8_t const*>(ptr);
    std::uint64_t h = seed ^ (len * m);
    std::size_t const numBlocks = len / 8;
    for (std::size_t i = 0; i < numBlocks; ++i)
    {
        auto k = detail::unalignedLoad<std::uint64_t>(data + i * 8);
        k *= m;
        k ^= k >> r;
        k *= m;
        h ^= k;
        h *= m;
    }
    auto const* const tail = data + numBlocks * 8;
    std::size_t const tailLen = len & 7U;
    if (tailLen != 0)
    {
        for (std::size_t i = 0; i < tailLen; ++i)
        {
            h ^= static_cast<std::uint64_t>(tail[i]) << (8U * i);
        }
        h *= m;
    }
    h ^= h >> r;
    // the final avalanche step is left to the table's keyToSlot
    return static_cast<std::size_t>(h);
}
inline std::size_t hash_int(std::uint64_t x) noexcept
{
    x ^= x >> 33U;
    x *= 0xff51afd7ed558ccdULL;
    x ^= x >> 33U;
    return static_cast<std::size_t>(x);
}
template <typename T>
struct hash : std::hash<T>
{
    std::size_t operator()(T const& obj) const noexcept(std::is_nothrow_invocable_v<std::hash<T> const&, T const&>)
    {
        return hash_int(static_cast<std::size_t>(std::hash<T>::operator()(obj)));
    }
};
template <typename CharT>
struct hash<std::basic_string<CharT>>
{
    std::size_t operator()(std::basic_string<CharT> const& str) const noexcept
    {
        return hash_bytes(str.data(), sizeof(CharT) * str.size());
    }
};
template <typename CharT>
struct hash<std::basic_string_view<CharT>>
{
    std::size_t operator()(std::basic_string_view<CharT> const& sv) const noexcept
    {
        return hash_bytes(sv.data(), sizeof(CharT) * sv.size());
    }
};
template <typename T>
struct hash<T*>
{
    std::size_t operator()(T* ptr) const noexcept
    {
        return hash_int(reinterpret_cast<std::uintptr_t>(ptr));
    }
};
template <typename T>
struct hash<std::unique_ptr<T>>
{
    std::size_t operator()(std::unique_ptr<T> const& ptr) const noexcept
    {
        return hash_int(reinterpret_cast<std::uintptr_t>(ptr.get()));
    }
};
template <typename T>
struct hash<std::shared_ptr<T>>
{
    std::size_t operator()(std::shared_ptr<T> const& ptr) const noexcept
    {
        return hash_int(reinterpret_cast<std::uintptr_t>(ptr.get()));
    }
};
template <typename Enum>
    requires std::is_enum_v<Enum>
struct hash<Enum>
{
    std::size_t operator()(Enum e) const noexcept
    {
        using Underlying = std::underlying_type_t<Enum>;
        return hash<Underlying>{}(static_cast<Underlying>(e));
    }
};
template <std::integral T>
struct hash<T>
{
    std::size_t operator()(T const& obj) const noexcept
    {
        return hash_int(static_cast<std::uint64_t>(obj));
    }
};
namespace detail
{
template <typename T>
concept HasIsTransparent = requires { typename T::is_transparent; };
// wrapper classes keep hash and key_equal from colliding when both are the same type
template <typename T>
struct WrapHash : T
{
    WrapHash() = default;
    explicit WrapHash(T const& o) noexcept(std::is_nothrow_copy_constructible_v<T>) :
        T(o)
    {
    }
};
template <typename T>
struct WrapKeyEqual : T
{
    WrapKeyEqual() = default;
    explicit WrapKeyEqual(T const& o) noexcept(std::is_nothrow_copy_constructible_v<T>) :
        T(o)
    {
    }
};
// Memory layout: [Node, Node, ... Node | info, info, ... infoSentinel]
// There are 2^n nodes plus an overflow buffer, and one info byte per node. An info byte of 0 means
// the node is empty, otherwise it holds the probe distance (in units of mInfoInc) plus a few hash bits.
// The sentinel byte is 1, so iterator increment stops at end() without an index.
// An unallocated table points mKeyVals and mInfo at its own mMask (always 0), so lookups on it
// need no special casing.
template <std::size_t MaxLoadFactor100, typename Key, typename T, typename Hash, typename KeyEqual>
class Table : public WrapHash<Hash>, public WrapKeyEqual<KeyEqual>
{
public:
    static constexpr bool is_map = !std::is_void_v<T>;
    static constexpr bool is_set = !is_map;
    static constexpr bool is_transparent = HasIsTransparent<Hash> && HasIsTransparent<KeyEqual>;
    using key_type = Key;
    using mapped_type = T;
    using value_type = std::conditional_t<is_map, robin_hood::pair<Key, T>, Key>;
    using size_type = std::size_t;
    using hasher = Hash;
    using key_equal = KeyEqual;
private:
    static_assert(MaxLoadFactor100 > 10 && MaxLoadFactor100 < 100, "MaxLoadFactor100 needs to be >10 && < 100");
    using WHash = WrapHash<Hash>;
    using WKeyEqual = WrapKeyEqual<KeyEqual>;
    using Node = value_type;
    using InfoType = std::uint32_t;
    static constexpr std::size_t InitialNumElements = sizeof(std::uint64_t);
    static constexpr std::uint32_t InitialInfoNumBits = 5;
    static constexpr std::uint8_t InitialInfoInc = 1U << InitialInfoNumBits;
    static constexpr std::size_t InfoMask = InitialInfoInc - 1U;
    static constexpr std::uint8_t InitialInfoHashShift = 0;
    struct Slot
    {
        std::size_t idx;
        InfoType info;
    };
    enum class InsertionState
    {
        overflow_error,
        key_found,
        new_node,
        overwrite_node
    };
    struct Insertion
    {
        std::size_t idx;
        InsertionState state;
    };
    struct fast_forward_tag
    {
    };
    template <bool IsConst>
    class Iter
    {
    private:
        using NodePtr = std::conditional_t<IsConst, Node const*, Node*>;
    public:
        using difference_type = std::ptrdiff_t;
        using value_type = typename Table::value_type;
        using reference = std::conditional_t<IsConst, value_type const&, value_type&>;
        using pointer = std::conditional_t<IsConst, value_type const*, value_type*>;
        using iterator_category = std::forward_iterator_tag;
        Iter() = default;
        template <bool OtherIsConst>
            requires(IsConst && !OtherIsConst)
        Iter(Iter<OtherIsConst> const& other) noexcept :
            mKeyVals(other.mKeyVals),
            mInfo(other.mInfo)
        {
        }
        Iter(NodePtr valPtr, std::uint8_t const* infoPtr) noexcept :
            mKeyVals(valPtr),
            mInfo(infoPtr)
        {
        }
        Iter(NodePtr valPtr, std::uint8_t const* infoPtr, fast_forward_tag) noexcept :
            mKeyVals(valPtr),
            mInfo(infoPtr)
        {
            fastForward();
        }
        Iter& operator++() noexcept
        {
            ++mInfo;
            ++mKeyVals;
            fastForward();
            return *this;
        }
        Iter operator++(int) noexcept
        {
            Iter tmp = *this;
            ++(*this);
            return tmp;
        }
        reference operator*() const
        {
            return *mKeyVals;
        }
        pointer operator->() const
        {
            return mKeyVals;
        }
        friend bool operator==(Iter const& a, Iter const& b) noexcept
        {
            return a.mKeyVals == b.mKeyVals;
        }
    private:
        // the info array is padded so that reading 8 bytes at any node position is safe
        void fastForward() noexcept
        {
            std::size_t n = 0;
            while ((n = detail::unalignedLoad<std::size_t>(mInfo)) == 0)
            {
                mInfo += sizeof(std::size_t);
                mKeyVals += sizeof(std::size_t);
            }
            auto const inc = (std::endian::native == std::endian::little ? std::countr_zero(n) : std::countl_zero(n)) / 8;
            mInfo += inc;
            mKeyVals += inc;
        }
        template <bool>
        friend class Iter;
        friend Table;
        NodePtr mKeyVals{ nullptr };
        std::uint8_t const* mInfo{ nullptr };
    };
public:
    using iterator = Iter<false>;
    using const_iterator = Iter<true>;
    Table() noexcept(std::is_nothrow_default_constructible_v<Hash> && std::is_nothrow_default_constructible_v<KeyEqual>) :
        WHash(),
        WKeyEqual()
    {
    }
    // nothing is allocated until the first insert; bucket_count is ignored
    explicit Table(std::size_t, Hash const& h = Hash{}, KeyEqual const& equal = KeyEqual{}) noexcept(std::is_nothrow_copy_constructible_v<Hash> && std::is_nothrow_copy_constructible_v<KeyEqual>) :
        WHash(h),
        WKeyEqual(equal)
    {
    }
    template <std::input_iterator It>
    Table(It first, It last, std::size_t = 0, Hash const& h = Hash{}, KeyEqual const& equal = KeyEqual{}) :
        WHash(h),
        WKeyEqual(equal)
    {
        insert(first, last);
    }
    Table(std::initializer_list<value_type> initlist, std::size_t = 0, Hash const& h = Hash{}, KeyEqual const& equal = KeyEqual{}) :
        WHash(h),
        WKeyEqual(equal)
    {
        insert(initlist.begin(), initlist.end());
    }
    Table(Table&& o) noexcept :
        WHash(std::move(static_cast<WHash&>(o))),
        WKeyEqual(std::move(static_cast<WKeyEqual&>(o)))
    {
        if (o.mMask != 0)
        {
            takeStorage(o);
        }
    }
    Table(Table const& o) :
        WHash(static_cast<WHash const&>(o)),
        WKeyEqual(static_cast<WKeyEqual const&>(o))
    {
        if (!o.empty())
        {
            allocateStorage(calcNumElementsWithBuffer(o.mMask + 1));
            copyState(o);
        }
    }
    Table& operator=(Table&& o) noexcept
    {
        if (&o == this)
        {
            return *this;
        }
        if (o.mMask != 0)
        {
            destroy();
            WHash::operator=(std::move(static_cast<WHash&>(o)));
            WKeyEqual::operator=(std::move(static_cast<WKeyEqual&>(o)));
            takeStorage(o);
        }
        else
        {
            clear();
        }
        return *this;
    }
    Table& operator=(Table const& o)
    {
        if (&o == this)
        {
            return *this;
        }
        if (o.empty())
        {
            if (mMask == 0)
            {
                return *this;
            }
            destroy();
            init();
            WHash::operator=(static_cast<WHash const&>(o));
            WKeyEqual::operator=(static_cast<WKeyEqual const&>(o));
            return *this;
        }
        destroyNodes();
        if (mMask != o.mMask)
        {
            if (mMask != 0)
            {
                freeStorage();
                init();
            }
            allocateStorage(calcNumElementsWithBuffer(o.mMask + 1));
        }
        WHash::operator=(static_cast<WHash const&>(o));
        WKeyEqual::operator=(static_cast<WKeyEqual const&>(o));
        copyState(o);
        return *this;
    }
    ~Table()
    {
        destroy();
    }
    void swap(Table& o)
    {
        std::swap(*this, o);
    }
    void clear()
    {
        if (empty())
        {
            return;
        }
        destroyNodes();
        auto const numElementsWithBuffer = calcNumElementsWithBuffer(mMask + 1);
        std::fill(mInfo, mInfo + calcNumBytesInfo(numElementsWithBuffer), std::uint8_t{ 0 });
        mInfo[numElementsWithBuffer] = 1;
        mInfoInc = InitialInfoInc;
        mInfoHashShift = InitialInfoHashShift;
    }
    bool operator==(Table const& other) const
    {
        if (other.size() != size())
        {
            return false;
        }
        for (auto const& otherEntry : other)
        {
            if (!has(otherEntry))
            {
                return false;
            }
        }
        return true;
    }
    auto& operator[](key_type const& key) requires is_map
    {
        return try_emplace(key).first->second;
    }
    auto& operator[](key_type&& key) requires is_map
    {
        return try_emplace(std::move(key)).first->second;
    }
    template <std::input_iterator It>
    void insert(It first, It last)
    {
        for (; first != last; ++first)
        {
            insert(value_type(*first));
        }
    }
    void insert(std::initializer_list<value_type> ilist)
    {
        insert(ilist.begin(), ilist.end());
    }
    template <typename... Args>
    std::pair<iterator, bool> emplace(Args&&... args)
    {
        value_type n(std::forward<Args>(args)...);
        auto const ins = insertKeyPrepareEmptySpot(keyOf(n));
        if (ins.state == InsertionState::overflow_error)
        {
            throwOverflowError();
        }
        if (ins.state != InsertionState::key_found)
        {
            placeNode(ins, std::move(n));
        }
        return makeResult(ins);
    }
    template <typename... Args>
    iterator emplace_hint(const_iterator, Args&&... args)
    {
        return emplace(std::forward<Args>(args)...).first;
    }
    template <typename... Args>
    std::pair<iterator, bool> try_emplace(key_type const& key, Args&&... args)
    {
        return tryEmplaceImpl(key, std::forward<Args>(args)...);
    }
    template <typename... Args>
    std::pair<iterator, bool> try_emplace(key_type&& key, Args&&... args)
    {
        return tryEmplaceImpl(std::move(key), std::forward<Args>(args)...);
    }
    template <typename... Args>
    iterator try_emplace(const_iterator, key_type const& key, Args&&... args)
    {
        return tryEmplaceImpl(key, std::forward<Args>(args)...).first;
    }
    template <typename... Args>
    iterator try_emplace(const_iterator, key_type&& key, Args&&... args)
    {
        return tryEmplaceImpl(std::move(key), std::forward<Args>(args)...).first;
    }
    template <typename Mapped>
    std::pair<iterator, bool> insert_or_assign(key_type const& key, Mapped&& obj)
    {
        return insertOrAssignImpl(key, std::forward<Mapped>(obj));
    }
    template <typename Mapped>
    std::pair<iterator, bool> insert_or_assign(key_type&& key, Mapped&& obj)
    {
        return insertOrAssignImpl(std::move(key), std::forward<Mapped>(obj));
    }
    template <typename Mapped>
    iterator insert_or_assign(const_iterator, key_type const& key, Mapped&& obj)
    {
        return insertOrAssignImpl(key, std::forward<Mapped>(obj)).first;
    }
    template <typename Mapped>
    iterator insert_or_assign(const_iterator, key_type&& key, Mapped&& obj)
    {
        return insertOrAssignImpl(std::move(key), std::forward<Mapped>(obj)).first;
    }
    std::pair<iterator, bool> insert(value_type const& keyval)
    {
        return emplace(keyval);
    }
    iterator insert(const_iterator, value_type const& keyval)
    {
        return emplace(keyval).first;
    }
    std::pair<iterator, bool> insert(value_type&& keyval)
    {
        return emplace(std::move(keyval));
    }
    iterator insert(const_iterator, value_type&& keyval)
    {
        return emplace(std::move(keyval)).first;
    }
    [[nodiscard]] std::size_t count(key_type const& key) const
    {
        return contains(key) ? 1U : 0U;
    }
    template <typename OtherKey>
        requires is_transparent
    [[nodiscard]] std::size_t count(OtherKey const& key) const
    {
        return contains(key) ? 1U : 0U;
    }
    [[nodiscard]] bool contains(key_type const& key) const
    {
        return findIdx(key) != endIdx();
    }
    template <typename OtherKey>
        requires is_transparent
    [[nodiscard]] bool contains(OtherKey const& key) const
    {
        return findIdx(key) != endIdx();
    }
    [[nodiscard]] auto& at(key_type const& key) requires is_map
    {
        auto const idx = findIdx(key);
        if (idx == endIdx()) [[unlikely]]
        {
            detail::doThrow<std::out_of_range>("key not found");
        }
        return mKeyVals[idx].second;
    }
    [[nodiscard]] auto const& at(key_type const& key) const requires is_map
    {
        auto const idx = findIdx(key);
        if (idx == endIdx()) [[unlikely]]
        {
            detail::doThrow<std::out_of_range>("key not found");
        }
        return mKeyVals[idx].second;
    }
    [[nodiscard]] const_iterator find(key_type const& key) const
    {
        auto const idx = findIdx(key);
        return const_iterator{ mKeyVals + idx, mInfo + idx };
    }
    template <typename OtherKey>
    [[nodiscard]] const_iterator find(OtherKey const& key, is_transparent_tag) const
    {
        auto const idx = findIdx(key);
        return const_iterator{ mKeyVals + idx, mInfo + idx };
    }
    template <typename OtherKey>
        requires is_transparent
    [[nodiscard]] const_iterator find(OtherKey const& key) const
    {
        auto const idx = findIdx(key);
        return const_iterator{ mKeyVals + idx, mInfo + idx };
    }
    [[nodiscard]] iterator find(key_type const& key)
    {
        auto const idx = findIdx(key);
        return iterator{ mKeyVals + idx, mInfo + idx };
    }
    template <typename OtherKey>
    [[nodiscard]] iterator find(OtherKey const& key, is_transparent_tag)
    {
        auto const idx = findIdx(key);
        return iterator{ mKeyVals + idx, mInfo + idx };
    }
    template <typename OtherKey>
        requires is_transparent
    [[nodiscard]] iterator find(OtherKey const& key)
    {
        auto const idx = findIdx(key);
        return iterator{ mKeyVals + idx, mInfo + idx };
    }
    iterator begin()
    {
        if (empty())
        {
            return end();
        }
        return iterator(mKeyVals, mInfo, fast_forward_tag{});
    }
    const_iterator begin() const
    {
        return cbegin();
    }
    const_iterator cbegin() const
    {
        if (empty())
        {
            return cend();
        }
        return const_iterator(mKeyVals, mInfo, fast_forward_tag{});
    }
    iterator end()
    {
        return iterator{ endNode(), nullptr };
    }
    const_iterator end() const
    {
        return cend();
    }
    const_iterator cend() const
    {
        return const_iterator{ endNode(), nullptr };
    }
    iterator erase(const_iterator pos)
    {
        return erase(iterator{ const_cast<Node*>(pos.mKeyVals), const_cast<std::uint8_t*>(pos.mInfo) });
    }
    iterator erase(iterator pos)
    {
        auto const idx = static_cast<std::size_t>(pos.mKeyVals - mKeyVals);
        shiftDown(idx);
        --mNumElements;
        if (*pos.mInfo != 0)
        {
            return pos;
        }
        return ++pos;
    }
    std::size_t erase(key_type const& key)
    {
        auto slot = keyToSlot(key);
        do
        {
            if (slot.info == mInfo[slot.idx] && WKeyEqual::operator()(key, keyOf(mKeyVals[slot.idx])))
            {
                shiftDown(slot.idx);
                --mNumElements;
                return 1;
            }
            next(slot);
        }
        while (slot.info <= mInfo[slot.idx]);
        return 0;
    }
    // same as reserve(c), but always rehashes; rehash(0) shrinks to fit
    void rehash(std::size_t c)
    {
        reserveImpl(c, true);
    }
    void reserve(std::size_t c)
    {
        reserveImpl(c, false);
    }
    void compact()
    {
        auto newSize = InitialNumElements;
        while (calcMaxNumElementsAllowed(newSize) < mNumElements && newSize != 0)
        {
            newSize *= 2;
        }
        if (newSize == 0) [[unlikely]]
        {
            throwOverflowError();
        }
        if (newSize < mMask + 1)
        {
            rehashPowerOfTwo(newSize);
        }
    }
    [[nodiscard]] size_type size() const noexcept
    {
        return mNumElements;
    }
    [[nodiscard]] size_type max_size() const noexcept
    {
        return std::numeric_limits<size_type>::max();
    }
    [[nodiscard]] bool empty() const noexcept
    {
        return mNumElements == 0;
    }
    [[nodiscard]] float max_load_factor() const noexcept
    {
        return MaxLoadFactor100 / 100.0F;
    }
    [[nodiscard]] float load_factor() const noexcept
    {
        return static_cast<float>(size()) / static_cast<float>(mMask + 1);
    }
    [[nodiscard]] std::size_t mask() const noexcept
    {
        return mMask;
    }
private:
    [[noreturn]] static void throwOverflowError()
    {
        detail::doThrow<std::overflow_error>("robin_hood::map overflow");
    }
    [[nodiscard]] static key_type const& keyOf(value_type const& v) noexcept
    {
        if constexpr (is_map)
        {
            return v.first;
        }
        else
        {
            return v;
        }
    }
    [[nodiscard]] static std::size_t calcMaxNumElementsAllowed(std::size_t maxElements) noexcept
    {
        if (maxElements <= std::numeric_limits<std::size_t>::max() / 100) [[likely]]
        {
            return maxElements * MaxLoadFactor100 / 100;
        }
        return (maxElements / 100) * MaxLoadFactor100;
    }
    // room for the sentinel byte plus padding, so 64-bit loads never run past the array
    [[nodiscard]] static std::size_t calcNumBytesInfo(std::size_t numElements) noexcept
    {
        return numElements + sizeof(std::uint64_t);
    }
public:
    [[nodiscard]] static std::size_t calcNumElementsWithBuffer(std::size_t numElements) noexcept
    {
        return numElements + (std::min)(calcMaxNumElementsAllowed(numElements), static_cast<std::size_t>(0xFF));
    }
    [[nodiscard]] static std::size_t calcNumBytesTotal(std::size_t numElements)
    {
        auto const infoBytes = calcNumBytesInfo(numElements);
        if (numElements > (std::numeric_limits<std::size_t>::max() - infoBytes) / sizeof(Node)) [[unlikely]]
        {
            throwOverflowError();
        }
        return numElements * sizeof(Node) + infoBytes;
    }
private:
    [[nodiscard]] Node* endNode() const noexcept
    {
        return detail::castPtr<Node>(mInfo);
    }
    [[nodiscard]] std::size_t endIdx() const noexcept
    {
        return static_cast<std::size_t>(endNode() - mKeyVals);
    }
    // lower bits index into the array, the upper InitialInfoNumBits bits are kept as a cheap pre-filter
    template <typename K>
    [[nodiscard]] Slot keyToSlot(K const& key) const
    {
        auto h = static_cast<std::uint64_t>(WHash::operator()(key));
        h *= mHashMultiplier;
        h ^= h >> 33U;
        return Slot{ (static_cast<std::size_t>(h) >> InitialInfoNumBits) & mMask, mInfoInc + static_cast<InfoType>((h & InfoMask) >> mInfoHashShift) };
    }
    void next(Slot& slot) const noexcept
    {
        ++slot.idx;
        slot.info += mInfoInc;
    }
    void nextWhileLess(Slot& slot) const noexcept
    {
        while (slot.info < mInfo[slot.idx])
        {
            next(slot);
        }
    }
    template <typename K>
    [[nodiscard]] bool matches(Slot const& slot, K const& key) const
    {
        return slot.info == mInfo[slot.idx] && WKeyEqual::operator()(key, keyOf(mKeyVals[slot.idx]));
    }
    template <typename K>
    [[nodiscard]] std::size_t findIdx(K const& key) const
    {
        auto slot = keyToSlot(key);
        do
        {
            if (matches(slot, key)) [[likely]]
            {
                return slot.idx;
            }
            next(slot);
            if (matches(slot, key)) [[likely]]
            {
                return slot.idx;
            }
            next(slot);
        }
        while (slot.info <= mInfo[slot.idx]);
        return endIdx();
    }
    void shiftUp(std::size_t startIdx, std::size_t insertionIdx) noexcept(std::is_nothrow_move_assignable_v<Node>)
    {
        auto idx = startIdx;
        std::construct_at(mKeyVals + idx, std::move(mKeyVals[idx - 1]));
        while (--idx != insertionIdx)
        {
            mKeyVals[idx] = std::move(mKeyVals[idx - 1]);
        }
        idx = startIdx;
        while (idx != insertionIdx)
        {
            mInfo[idx] = static_cast<std::uint8_t>(mInfo[idx - 1] + mInfoInc);
            if (mInfo[idx] + mInfoInc > 0xFF) [[unlikely]]
            {
                mMaxNumElementsAllowed = 0;
            }
            --idx;
        }
    }
    void shiftDown(std::size_t idx) noexcept(std::is_nothrow_move_assignable_v<Node>)
    {
        while (mInfo[idx + 1] >= 2 * mInfoInc)
        {
            mInfo[idx] = static_cast<std::uint8_t>(mInfo[idx + 1] - mInfoInc);
            mKeyVals[idx] = std::move(mKeyVals[idx + 1]);
            ++idx;
        }
        mInfo[idx] = 0;
        std::destroy_at(mKeyVals + idx);
    }
    // the node is guaranteed to be new, e.g. while rehashing
    void insertMove(Node&& keyval)
    {
        if (mMaxNumElementsAllowed == 0 && !tryIncreaseInfo())
        {
            throwOverflowError();
        }
        auto slot = keyToSlot(keyOf(keyval));
        while (slot.info <= mInfo[slot.idx])
        {
            next(slot);
        }
        auto const insertionIdx = slot.idx;
        auto const insertionInfo = static_cast<std::uint8_t>(slot.info);
        if (insertionInfo + mInfoInc > 0xFF) [[unlikely]]
        {
            mMaxNumElementsAllowed = 0;
        }
        while (mInfo[slot.idx] != 0)
        {
            next(slot);
        }
        if (slot.idx == insertionIdx)
        {
            std::construct_at(mKeyVals + insertionIdx, std::move(keyval));
        }
        else
        {
            shiftUp(slot.idx, insertionIdx);
            mKeyVals[insertionIdx] = std::move(keyval);
        }
        mInfo[insertionIdx] = insertionInfo;
        ++mNumElements;
    }
    // Finds the key. If it is absent, shifts nodes out of the way and updates mInfo and the element
    // count, so all that is left to do is to construct or assign a node at the returned index.
    template <typename K>
    [[nodiscard]] Insertion insertKeyPrepareEmptySpot(K const& key)
    {
        for (int i = 0; i < 256; ++i)
        {
            auto slot = keyToSlot(key);
            nextWhileLess(slot);
            while (slot.info == mInfo[slot.idx])
            {
                if (WKeyEqual::operator()(key, keyOf(mKeyVals[slot.idx])))
                {
                    return { slot.idx, InsertionState::key_found };
                }
                next(slot);
            }
            if (mNumElements >= mMaxNumElementsAllowed) [[unlikely]]
            {
                increaseSize();
                continue;
            }
            auto const insertionIdx = slot.idx;
            auto const insertionInfo = slot.info;
            if (insertionInfo + mInfoInc > 0xFF) [[unlikely]]
            {
                mMaxNumElementsAllowed = 0;
            }
            while (mInfo[slot.idx] != 0)
            {
                next(slot);
            }
            if (slot.idx != insertionIdx)
            {
                shiftUp(slot.idx, insertionIdx);
            }
            mInfo[insertionIdx] = static_cast<std::uint8_t>(insertionInfo);
            ++mNumElements;
            return { insertionIdx, slot.idx == insertionIdx ? InsertionState::new_node : InsertionState::overwrite_node };
        }
        return { 0, InsertionState::overflow_error };
    }
    void placeNode(Insertion ins, value_type&& v)
    {
        if (ins.state == InsertionState::new_node)
        {
            std::construct_at(mKeyVals + ins.idx, std::move(v));
        }
        else
        {
            mKeyVals[ins.idx] = std::move(v);
        }
    }
    template <typename... Args>
    void placeNode(Insertion ins, Args&&... args)
    {
        if (ins.state == InsertionState::new_node)
        {
            std::construct_at(mKeyVals + ins.idx, std::forward<Args>(args)...);
        }
        else
        {
            mKeyVals[ins.idx] = value_type(std::forward<Args>(args)...);
        }
    }
    [[nodiscard]] std::pair<iterator, bool> makeResult(Insertion ins) noexcept
    {
        return { iterator(mKeyVals + ins.idx, mInfo + ins.idx), ins.state != InsertionState::key_found };
    }
    template <typename OtherKey, typename... Args>
    std::pair<iterator, bool> tryEmplaceImpl(OtherKey&& key, Args&&... args)
    {
        auto const ins = insertKeyPrepareEmptySpot(key);
        if (ins.state == InsertionState::overflow_error)
        {
            throwOverflowError();
        }
        if (ins.state != InsertionState::key_found)
        {
            placeNode(ins, std::piecewise_construct, std::forward_as_tuple(std::forward<OtherKey>(key)), std::forward_as_tuple(std::forward<Args>(args)...));
        }
        return makeResult(ins);
    }
    template <typename OtherKey, typename Mapped>
    std::pair<iterator, bool> insertOrAssignImpl(OtherKey&& key, Mapped&& obj)
    {
        auto const ins = insertKeyPrepareEmptySpot(key);
        if (ins.state == InsertionState::overflow_error)
        {
            throwOverflowError();
        }
        if (ins.state == InsertionState::key_found)
        {
            mKeyVals[ins.idx].second = std::forward<Mapped>(obj);
        }
        else
        {
            placeNode(ins, std::piecewise_construct, std::forward_as_tuple(std::forward<OtherKey>(key)), std::forward_as_tuple(std::forward<Mapped>(obj)));
        }
        return makeResult(ins);
    }
    [[nodiscard]] bool has(value_type const& e) const
    {
        auto const it = find(keyOf(e));
        if constexpr (is_map)
        {
            return it != end() && it->second == e.second;
        }
        else
        {
            return it != end();
        }
    }
    void reserveImpl(std::size_t c, bool forceRehash)
    {
        auto const minElementsAllowed = (std::max)(c, mNumElements);
        auto newSize = InitialNumElements;
        while (calcMaxNumElementsAllowed(newSize) < minElementsAllowed && newSize != 0)
        {
            newSize *= 2;
        }
        if (newSize == 0) [[unlikely]]
        {
            throwOverflowError();
        }
        if (forceRehash || newSize > mMask + 1)
        {
            rehashPowerOfTwo(newSize);
        }
    }
    void rehashPowerOfTwo(std::size_t numBuckets)
    {
        Node* const oldKeyVals = mKeyVals;
        std::uint8_t const* const oldInfo = mInfo;
        auto const oldNumElementsWithBuffer = calcNumElementsWithBuffer(mMask + 1);
        initData(numBuckets);
        if (oldNumElementsWithBuffer > 1)
        {
            for (std::size_t i = 0; i < oldNumElementsWithBuffer; ++i)
            {
                if (oldInfo[i] != 0)
                {
                    insertMove(std::move(oldKeyVals[i]));
                    std::destroy_at(oldKeyVals + i);
                }
            }
            detail::freeBytes<Node>(oldKeyVals);
        }
    }
    bool tryIncreaseInfo()
    {
        if (mInfoInc <= 2)
        {
            return false;
        }
        mInfoInc = static_cast<InfoType>(mInfoInc >> 1U);
        ++mInfoHashShift;
        auto const numElementsWithBuffer = calcNumElementsWithBuffer(mMask + 1);
        // drops one hash bit from every info byte, 8 bytes at a time
        for (std::size_t i = 0; i < numElementsWithBuffer; i += 8)
        {
            auto val = detail::unalignedLoad<std::uint64_t>(mInfo + i);
            val = (val >> 1U) & 0x7f7f7f7f7f7f7f7fULL;
            std::memcpy(mInfo + i, &val, sizeof(val));
        }
        mInfo[numElementsWithBuffer] = 1;
        mMaxNumElementsAllowed = calcMaxNumElementsAllowed(mMask + 1);
        return true;
    }
    void increaseSize()
    {
        if (mMask == 0)
        {
            initData(InitialNumElements);
            return;
        }
        if (mNumElements < calcMaxNumElementsAllowed(mMask + 1) && tryIncreaseInfo())
        {
            return;
        }
        if (mNumElements * 2 < calcMaxNumElementsAllowed(mMask + 1))
        {
            // plenty of room left, so the info overflow is blamed on a bad multiplier
            nextHashMultiplier();
            rehashPowerOfTwo(mMask + 1);
        }
        else
        {
            rehashPowerOfTwo((mMask + 1) * 2);
        }
    }
    // adding an even number keeps the multiplier odd, so the multiplication stays a bijection
    void nextHashMultiplier()
    {
        mHashMultiplier += 0xc4ceb9fe1a85ec54ULL;
    }
    void allocateStorage(std::size_t numElementsWithBuffer)
    {
        mKeyVals = static_cast<Node*>(detail::allocateBytes<Node>(calcNumBytesTotal(numElementsWithBuffer)));
        mInfo = detail::castPtr<std::uint8_t>(mKeyVals + numElementsWithBuffer);
    }
    void freeStorage() noexcept
    {
        detail::freeBytes<Node>(mKeyVals);
    }
    void initData(std::size_t maxElements)
    {
        auto const numElementsWithBuffer = calcNumElementsWithBuffer(maxElements);
        auto* const newKeyVals = static_cast<Node*>(detail::allocateBytes<Node>(calcNumBytesTotal(numElementsWithBuffer)));
        mKeyVals = newKeyVals;
        mInfo = detail::castPtr<std::uint8_t>(newKeyVals + numElementsWithBuffer);
        std::memset(mInfo, 0, calcNumBytesInfo(numElementsWithBuffer));
        mInfo[numElementsWithBuffer] = 1;
        mNumElements = 0;
        mMask = maxElements - 1;
        mMaxNumElementsAllowed = calcMaxNumElementsAllowed(maxElements);
        mInfoInc = InitialInfoInc;
        mInfoHashShift = InitialInfoHashShift;
    }
    void takeStorage(Table& o) noexcept
    {
        mHashMultiplier = o.mHashMultiplier;
        mKeyVals = o.mKeyVals;
        mInfo = o.mInfo;
        mNumElements = o.mNumElements;
        mMask = o.mMask;
        mMaxNumElementsAllowed = o.mMaxNumElementsAllowed;
        mInfoInc = o.mInfoInc;
        mInfoHashShift = o.mInfoHashShift;
        o.init();
    }
    // storage for o's layout must already be allocated
    void copyState(Table const& o)
    {
        mHashMultiplier = o.mHashMultiplier;
        mNumElements = o.mNumElements;
        mMask = o.mMask;
        mMaxNumElementsAllowed = o.mMaxNumElementsAllowed;
        mInfoInc = o.mInfoInc;
        mInfoHashShift = o.mInfoHashShift;
        cloneData(o);
    }
    void cloneData(Table const& o)
    {
        auto const numElementsWithBuffer = calcNumElementsWithBuffer(mMask + 1);
        if constexpr (std::is_trivially_copyable_v<Node>)
        {
            std::memcpy(mKeyVals, o.mKeyVals, calcNumBytesTotal(numElementsWithBuffer));
        }
        else
        {
            std::copy(o.mInfo, o.mInfo + calcNumBytesInfo(numElementsWithBuffer), mInfo);
            for (std::size_t i = 0; i < numElementsWithBuffer; ++i)
            {
                if (mInfo[i] != 0)
                {
                    std::construct_at(mKeyVals + i, o.mKeyVals[i]);
                }
            }
        }
    }
    void destroyNodes() noexcept
    {
        mNumElements = 0;
        if constexpr (!std::is_trivially_destructible_v<Node>)
        {
            auto const numElementsWithBuffer = calcNumElementsWithBuffer(mMask + 1);
            for (std::size_t idx = 0; idx < numElementsWithBuffer; ++idx)
            {
                if (mInfo[idx] != 0)
                {
                    std::destroy_at(mKeyVals + idx);
                }
            }
        }
    }
    void destroy()
    {
        if (mMask == 0)
        {
            return;
        }
        destroyNodes();
        freeStorage();
    }
    void init() noexcept
    {
        mKeyVals = detail::castPtr<Node>(&mMask);
        mInfo = detail::castPtr<std::uint8_t>(&mMask);
        mNumElements = 0;
        mMask = 0;
        mMaxNumElementsAllowed = 0;
        mInfoInc = InitialInfoInc;
        mInfoHashShift = InitialInfoHashShift;
    }
    std::uint64_t mHashMultiplier = 0xc4ceb9fe1a85ec53ULL;
    Node* mKeyVals = detail::castPtr<Node>(&mMask);
    std::uint8_t* mInfo = detail::castPtr<std::uint8_t>(&mMask);
    std::size_t mNumElements = 0;
    std::size_t mMask = 0;
    std::size_t mMaxNumElementsAllowed = 0;
    InfoType mInfoInc = InitialInfoInc;
    InfoType mInfoHashShift = InitialInfoHashShift;
};
}
template <typename Key, typename T, typename Hash = hash<Key>, typename KeyEqual = std::equal_to<Key>, std::size_t MaxLoadFactor100 = 80>
using unordered_flat_map = detail::Table<MaxLoadFactor100, Key, T, Hash, KeyEqual>;
template <typename Key, typename Hash = hash<Key>, typename KeyEqual = std::equal_to<Key>, std::size_t MaxLoadFactor100 = 80>
using unordered_flat_set = detail::Table<MaxLoadFactor100, Key, void, Hash, KeyEqual>;
}
// structured bindings for robin_hood::pair; the const variants are provided by std
namespace std
{
template <typename T1, typename T2>
struct tuple_size<robin_hood::pair<T1, T2>> : integral_constant<size_t, 2>
{
};
template <size_t I, typename T1, typename T2>
struct tuple_element<I, robin_hood::pair<T1, T2>> : tuple_element<I, tuple<T1, T2>>
{
};
}
#undef ROBIN_HOOD_NOINLINE
