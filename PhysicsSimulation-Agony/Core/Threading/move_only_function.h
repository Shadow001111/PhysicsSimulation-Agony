#pragma once
#include <functional>
#include <memory>
#include <type_traits>
#include <concepts>

namespace Core::Threading
{
    template <class>
    class move_only_function;

    template <class R, class... Args>
    class move_only_function<R(Args...)>
    {
        static constexpr size_t SBO_SIZE = 4 * sizeof(void*);
        static constexpr size_t SBO_ALIGN = alignof(std::max_align_t);

        struct base
        {
            virtual ~base() = default;
            virtual R call(Args&&... args) = 0;
            virtual base* move_to(void* where) const = 0;
        };

        template <class F>
        struct model final : base
        {
            F f;

            template <class U>
            explicit model(U&& u) : f(std::forward<U>(u)) {}

            R call(Args&&... args) override
            {
                if constexpr (std::is_void_v<R>)
                {
                    std::invoke(f, std::forward<Args>(args)...);
                }
                else
                {
                    return std::invoke(f, std::forward<Args>(args)...);
                }
            }

            base* move_to(void* where) const override
            {
                return new (where) model<F>(std::move(f));
            }
        };


        alignas(SBO_ALIGN) std::byte sbo_[SBO_SIZE];
        base* ptr_ = nullptr;


        void clear() noexcept
        {
            if (ptr_)
            {
                if (ptr_ == reinterpret_cast<base*>(sbo_))
                {
                    ptr_->~base(); // In-place destruct.
                }
                else
                {
                    delete ptr_; // Heap allocated.
                }
                ptr_ = nullptr;
            }
        }

        void move_from(move_only_function&& other) noexcept
        {
            if (other.ptr_ == reinterpret_cast<base*>(other.sbo_))
            {
                ptr_ = other.ptr_->move_to(sbo_);
                other.clear();
            }
            else
            {
                ptr_ = other.ptr_;
                other.ptr_ = nullptr;
            }
        }
    public:
        move_only_function() noexcept = default;
        move_only_function(std::nullptr_t) noexcept {}

        ~move_only_function() noexcept { clear(); }

        move_only_function(const move_only_function&) = delete;
        move_only_function& operator=(const move_only_function&) = delete;

        move_only_function(move_only_function&& other) noexcept
        {
            move_from(std::move(other));
        }

        move_only_function& operator=(move_only_function&& other) noexcept
        {
            if (this != &other)
            {
                clear();
                move_from(std::move(other));
            }
            return *this;
        }

        template <class F>
            requires (!std::same_as<std::remove_cvref_t<F>, move_only_function>&&
                       std::constructible_from<std::decay_t<F>, F>&&
                       std::invocable<std::decay_t<F>&, Args...>&&
                       std::is_invocable_r_v<R, std::decay_t<F>&, Args...>
                     )
            move_only_function(F&& f)
        {
            using Decayed = std::decay_t<F>;
            if constexpr (sizeof(model<Decayed>) <= SBO_SIZE && alignof(model<Decayed>) <= SBO_ALIGN)
            {   // Store in SBO.
                ptr_ = new (sbo_) model<Decayed>(std::forward<F>(f));
            }
            else
            {   // Store in heap.
                ptr_ = new model<Decayed>(std::forward<F>(f));
            }
        }

        explicit operator bool() const noexcept { return ptr_ != nullptr; }

        void reset() noexcept { clear(); }

        R operator()(Args... args)
        {
            if (!ptr_) throw std::bad_function_call{};
            return ptr_->call(std::forward<Args>(args)...);
        }
    };
}
