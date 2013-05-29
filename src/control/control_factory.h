#pragma once
#include "control.h"
// STL includes
#include <string>
#include <list>


//############################################################
//### Macro that automagically populate list of controllers
#define REGISTER_CONTROLLER(x, c) static control::control_factory_impl<x> __factory__##c (#c);

namespace control {

    class control_factory_t;
    class control_t;
    
    static const control_factory_t* get_factory(std::string name);
    static const std::list<control_factory_t*>& factories();

    class control_factory_t {
    public:
        control_factory_t(std::string name);
        virtual ~control_factory_t();

        virtual control_t* create() const = 0;
        virtual std::string name() const { return _name; }

        static const std::list<control_factory_t*>& factory_list();

    protected:
        std::string _name;
        static std::list<control_factory_t*> *_factories;
    };
    
    template<class X>
    class control_factory_impl : public control_factory_t {
    public:
        control_factory_impl(std::string name)
            : control_factory_t(name) {
        }

        virtual ~control_factory_impl() {
        }
        
        virtual control_t* create() const {
            control_t *controller = new X(name());
            return controller;
        }
    };

}
