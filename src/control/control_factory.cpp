#include "control_factory.h"
#include <iostream>
#include <boost/foreach.hpp>

namespace control {
    
    //############################################################
    //### Global variables and functions
    //############################################################
    std::list<control_factory_t*> *control_factory_t::_factories = 0;

    const control_factory_t* get_factory(std::string name)
    {
        BOOST_FOREACH(control_factory_t* factory, control_factory_t::factory_list()) {
            if(name == factory->name())
                return factory;
        }
        std::cerr << "Error: Factory " << name << " not found." << std::endl;
        return NULL;
    }
    
    const std::list<control_factory_t*>& factories() {
        return control_factory_t::factory_list();
    }

    const std::list<control_factory_t*>& control_factory_t::factory_list()
    {
        return *control_factory_t::_factories;
    }

    //############################################################
    //### Control factory
    //############################################################
    control_factory_t::control_factory_t(std::string name)
        : _name(name)
    {
        if(!_factories) {
            _factories = new std::list<control_factory_t*>();
        }
        _factories->push_back(this);
    }

    control_factory_t::~control_factory_t() {}
}
