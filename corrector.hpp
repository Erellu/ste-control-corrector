/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, Erwan DUHAMEL
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef STE_CONTROL_CORRECTOR_HPP
#define STE_CONTROL_CORRECTOR_HPP

#include <algorithm>
#include <atomic>

#include <chrono>
#include <cstdint>

#include <iostream>

#include <memory>

#include <numeric>

#include <optional>


/// Standard Libary Extensions developped by Erellu. See the others here https://github.com/Erellu.
namespace ste
{

/// Control-theory-related classes and functions.
namespace control
{

/// Various modes available for correctors.
enum class correctors_mode
{
    p,   /**< Proportional (P) corrector. */
    i,   /**< Integral (I) corrector. */
    pi,  /**< Proportional-Integral (PI) corrector. */
    pid, /**< Proportional-Integral-Derivative (PID) corrector. */
};

inline std::ostream& operator<<(std::ostream& out, const correctors_mode mode)
{
    using m = correctors_mode;

    switch (mode)
    {
        case(m::p):{out << "p"; break;}
        case(m::i):{out << "i"; break;}
        case(m::pi):{out << "pi"; break;}
        case(m::pid):{out << "pid"; break;}
        default:{break;}
    }

    return out;
};

//-----------------------------------------------------------------------

/** @brief Basic discrete corrector.
 *  @tparam mode corrector mode.
 *  @tparam arithmetic_t Type used for arithmeticd. Default is float.
 *  @tparam sampling_t Sampling period type. Default is std::chrono::milliseconds.
 *  @tparam timestamp_t Timestamp type. Default is std::chrono::time_point<std::chrono::system_clock>.
 *
 * @details Notes about computations
 *    NOTATIONS :
 *    e(t) : error
 *    u(t) : output of the corrector
 *
 *    //------------------------------------------------------
 *
 *    P Corrector :
 *    u(t) = p * e(t)
 *
 *    //------------------------------------------------------
 *
 *    I Corrector :
 *    u(t) = (1/i)*integral(e(t))
 *
 *    //------------------------------------------------------
 *
 *    PI Corrector :
 *    u(t) = p*(e(t) + (1/i)*integral(e(t))
 *
 *    //------------------------------------------------------
 *
 *    PID Corrector :
 *    u(t) =  (p * e(t)) +
 *            (i * integral(e(t)) +
 *            (d * derivative(e(t))
 *
 *    //------------------------------------------------------
 *
 *    N.B. :
 *    1) Derivative is computed using slope approximation (new_error - previous_error) / sample_time.
 *    2) Integral is computed using rectangle method.
 *
 *    @example
 *
 *    #include <corrector.hpp>
 *
 *    #include <iostream>
 *
 *    int main()
 *    {
 *        ste::control::corrector<ste::control::correctors_mode::p> c(
 *                    {
 *                        .ts          = std::chrono::milliseconds(25), //Sampling period. Type can be adjusted by template parameter.
 *                        .p           = 5,                             //P factor
 *                        .i           = 1,                             //Ignored, mode is p
 *                        .d           = 0,                             //Idem
 *                        .saturations = {}                             //Default saturation values
 *                    });
 *
 *        std::cout << c << std::endl;
 *
 *        c.update(1,0); //Update the corrector output: the system input is 1 and its output is 0.
 *
 *        std::cout << c << std::endl;
 *
 *        c.reset(); //Reset the corrector.
 *
 *        std::cout << c << std::endl;
 *
 *        return 0;
 *    }
 */
template<correctors_mode mode,
         typename arithmetic_t = float,
         typename sampling_t   = std::chrono::milliseconds,
         typename timestamp_t  = std::chrono::time_point<std::chrono::system_clock>,
         typename = std::enable_if_t<std::is_arithmetic_v<arithmetic_t>>,
         typename = std::enable_if_t<std::is_arithmetic_v<sampling_t> ||
                                     std::is_same_v<sampling_t, std::chrono::nanoseconds> ||
                                     std::is_same_v<sampling_t, std::chrono::microseconds> ||
                                     std::is_same_v<sampling_t, std::chrono::milliseconds> ||
                                     std::is_same_v<sampling_t, std::chrono::seconds> ||
                                     std::is_same_v<sampling_t, std::chrono::minutes> ||
                                     std::is_same_v<sampling_t, std::chrono::hours>
                                     >,
         typename = std::enable_if_t<(mode == correctors_mode::p) ||
                                     (mode == correctors_mode::i) ||
                                     (mode == correctors_mode::pi) ||
                                     (mode == correctors_mode::pid)
                                     >
         >
class corrector
{
public:

    /*********************************************************************/
    /*                            Public types                           */
    /*********************************************************************/

    /// Structure wrapping information required to create the object.
    struct create_info_t
    {
        /// Wrapper for saturation values.
        struct saturation_t
        {
            /// Maximum value output by the corrector.
            const arithmetic_t max = std::numeric_limits<arithmetic_t>::max();

            /// Minimum value output by the corrector.
            const arithmetic_t min = -std::numeric_limits<arithmetic_t>::max();
        };

        /// Sampling period.
        const sampling_t ts;

        /// Proportional factor.
        const arithmetic_t p;

        /// Integral factor.
        const arithmetic_t i;

        /// Derivative factor.
        const arithmetic_t d;

        /// Saturation values.
        std::optional<saturation_t> saturations;
    };

    /*********************************************************************/
    /*                    Constructors / Destructor                      */
    /*********************************************************************/

    /// Constructor.
    inline corrector(const create_info_t& info) : _state {}
    {
        _parameters.ts = info.ts;
        _parameters.p  = info.p;
        _parameters.i  = info.i;
        _parameters.d  = info.d;

        const auto& saturations = info.saturations.value_or(typename create_info_t::saturation_t {} );
        _parameters.max = saturations.max;
        _parameters.min = saturations.min;
    }

    /*********************************************************************/
    /*                             Operators                             */
    /*********************************************************************/

    inline friend std::ostream& operator<<(std::ostream& out, const corrector& c)
    {

        return out << "ste::control::corrector " << std::addressof(c) << "\n"
                   << "    Mode: " << mode << "\n"
                   << "    Parameters:\n"
                   << "        ts (sampling period): "    << c.ts().count() << "\n"
                   << "        p (proportional factor): " << c.p()          << "\n"
                   << "        i (integral factor): "     << c.i()          << "\n"
                   << "        d (derivative factor): "   << c.d()          << "\n"
                   << "        max (upper saturation value): "  << c.max()        << "\n"
                   << "        min (lower saturation value): "  << c.min()        << "\n"
                   << "    State:\n"
                   << "        output: "    << c._state.output                << "\n"
                   << "        integral: "  << c._state.integral              << "\n"
                   << "        error: "     << c._state.error                 << "\n"
                   << "        output: "    << c._state.output                << "\n";
    }

    /*********************************************************************/
    /*                               Control                             */
    /*********************************************************************/

    /**
     *  @brief Updates the corrector ouput.
     *  @param system_input Current input for the system.
     *  @param system_output Current system output.
     */
    inline void update(const arithmetic_t system_input, const arithmetic_t system_output)
    {
        const arithmetic_t error = system_input - system_output;
        const arithmetic_t ts = static_cast<arithmetic_t>(this->ts().count());

        _state.integral += ts;
        _state.timestamp = static_cast<timestamp_t>(std::chrono::system_clock::now());

        const auto compute = [this, ts, error]() constexpr -> arithmetic_t
        {
            if constexpr (mode == correctors_mode::p)
            {
                return p() * error;
            }
            else if constexpr (mode == correctors_mode::i)
            {
                const arithmetic_t i = this->i();

                return (static_cast<arithmetic_t>(1)/i) * _state.integral;
            }
            else if constexpr (mode == correctors_mode::pi)
            {
                const arithmetic_t p = this->p();
                const arithmetic_t i = this->i();

                return (p) * (error + (1/i) * _state.integral);
            }
            else if constexpr (mode == correctors_mode::pid)
            {
                const arithmetic_t p  = this->p();
                const arithmetic_t i  = this->i();
                const arithmetic_t d  = this->d();

                return  (p * error) +
                        (i * _state.integral ) +
                        (d * (error - _state.error) / ts);
            }
            else
            {
                return arithmetic_t{}; //That cannot happen due to enable_if, but this silences a warning
            }
        };

        //Update the current states
        _state.output = compute();
        _state.error  = error;

        // Saturation
        if(_state.output > max()){_state.output = max();}
        if(_state.output < min()){_state.output = min();}
    }

    //----------------------------------------------------------------

    /**
     *  @brief Resets the corrector states.
     */
    inline void reset()
    {
        _state = state_t{};
    }

    /*********************************************************************/
    /*                         Accessors / Mutators                      */
    /*********************************************************************/

    /**
     *  @brief Sets the corrector sampling period.
     *  @param p New sampling period.
     */
    inline void set_ts(const sampling_t ts){_parameters.ts = ts;}

    //----------------------------------------------------------------

    /**
     *  @brief Sets the corrector proportional factor.
     *  @param p New proportional factor.
     */
    inline void set_p(const arithmetic_t p){_parameters.p = p;};

    //----------------------------------------------------------------

    /**
     *  @brief Sets the corrector integral factor.
     *  @param p New proportional factor.
     */
    inline void set_i(const arithmetic_t i){_parameters.i = i;};

    //----------------------------------------------------------------

    /**
     *  @brief Sets the corrector derivative factor.
     *  @param p New proportional factor.
     */
    inline void set_d(const arithmetic_t d){_parameters.d = d;};

    //----------------------------------------------------------------

    /**
     *  @brief Sets the corrector upper saturation value.
     *  @param max New upper saturation value.
     */
    inline void set_max(const arithmetic_t max){_parameters.max = max;};

    //----------------------------------------------------------------

    /**
     *  @brief Sets the corrector lower saturation value.
     *  @param min New lower saturation value.
     */
    inline void set_min(const arithmetic_t min){_parameters.min = min;};

    //----------------------------------------------------------------

    /**
     *  @brief Sets the corrector saturation values.
     *  @param min New lower saturation value.
     *  @param max New upper saturation value..
     */
    inline void set_range(const arithmetic_t min, const arithmetic_t max)
    {
        if(min > max)
        {
            return set_range(max, min); //Don't trust the user
        }

        _parameters.min = min;
        _parameters.max = max;
    }

    //----------------------------------------------------------------

    /**
     *  @brief Returns the corrector last timestamp (updated each time update is called).
     */
    inline timestamp_t timestamp() const{return _state.timestamp;}

    /**
     *  @brief Returns the corrector sampling period.
     */
    inline sampling_t ts() const{return _parameters.ts.load();}

    //----------------------------------------------------------------

    /**
     *  @brief Returns the corrector proportional factor.
     */
    inline arithmetic_t p() const{return _parameters.p.load();}

    //----------------------------------------------------------------

    /**
     *  @brief Returns the corrector integral factor.
     */
    inline arithmetic_t i() const{return _parameters.i.load();}

    //----------------------------------------------------------------

    /**
     *  @brief Returns the corrector derivative factor.
     */
    inline arithmetic_t d() const{return _parameters.d.load();}

    //----------------------------------------------------------------

    /**
     *  @brief Returns the corrector upper saturation value.
     */
    inline arithmetic_t max() const{return _parameters.max.load();}

    //----------------------------------------------------------------

    /**
     *  @brief Returns the corrector lower saturation value.
     */
    inline arithmetic_t min() const{return _parameters.min.load();}

    //----------------------------------------------------------------

    /**
     *  @brief Returns the current corrector output.
     */
    inline arithmetic_t output() const{return _state.output;}

private:

    /*********************************************************************/
    /*                           Private types                           */
    /*********************************************************************/

    /// Internal wrapper holding the corrector parameters.
    struct parameters_t
    {
        std::atomic<sampling_t> ts;

        std::atomic<arithmetic_t> p;
        std::atomic<arithmetic_t> i;
        std::atomic<arithmetic_t> d;

        std::atomic<arithmetic_t> max;
        std::atomic<arithmetic_t> min;
    };

    /// Internal wrapper holding the corrector current states.
    struct state_t
    {
        /// Current output
        arithmetic_t output = arithmetic_t{};

        /// Timestamp saving the last time update() was called.
        timestamp_t timestamp = timestamp_t{};

        /// Current integral value.
        arithmetic_t integral = arithmetic_t{};

        /// Previous error.
        arithmetic_t error = arithmetic_t{};
    };

    /*********************************************************************/
    /*                              Attributes                           */
    /*********************************************************************/

    /// Corrector parameters.
    parameters_t _parameters;

    /// Corrector states.
    state_t _state = {};
};


} //namespace control

} //namespace ste

#endif // STE_CONTROL_CORRECTOR_HPP
