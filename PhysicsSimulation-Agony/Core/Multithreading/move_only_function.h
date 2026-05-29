#pragma once
#include <functional>
#include <memory>
#include <type_traits>
#include <concepts>

template <class>
class move_only_function;

template <class R, class... Args>
class move_only_function<R(Args...)>
{
    struct base
    {
        virtual ~base() = default;
        virtual R call(Args&&... args) = 0;
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
    };

    std::unique_ptr<base> ptr;
public:
    move_only_function() noexcept = default;
    move_only_function(std::nullptr_t) noexcept {}

    move_only_function(const move_only_function&) = delete;
    move_only_function& operator=(const move_only_function&) = delete;

    move_only_function(move_only_function&&) noexcept = default;
    move_only_function& operator=(move_only_function&&) noexcept = default;

    template <class F>
        requires (!std::same_as<std::remove_cvref_t<F>, move_only_function>&&
                   std::constructible_from<std::decay_t<F>, F>&&
                   std::invocable<std::decay_t<F>&, Args...>&&
                   std::is_invocable_r_v<R, std::decay_t<F>&, Args...>)
        move_only_function(F&& f) :
        ptr(std::make_unique<model<std::decay_t<F>>>(std::forward<F>(f)))
    {
    }

    explicit operator bool() const noexcept { return static_cast<bool>(ptr); }

    void reset() noexcept { ptr.reset(); }

    R operator()(Args... args)
    {
        if (!ptr) throw std::bad_function_call{};
        return ptr->call(std::forward<Args>(args)...);
    }
};