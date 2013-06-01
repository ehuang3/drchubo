#include "control_factory.h"
#include <iostream>
#include <boost/foreach.hpp>

//############################################################
//### Register controllers below
#include "arms.h"
//REGISTER_CONTROLLER(control::ARM_AIK_T, ARM_AIK)
//REGISTER_CONTROLLER(control::ARM_AJIK_T, ARM_AJIK)
REGISTER_CONTROLLER(control::ARM_JIT_T, ARM_JIT)
REGISTER_CONTROLLER(control::ARM_BOTH_JIT_T, ARM_BOTH_JIT)
//REGISTER_CONTROLLER(control::ARM_JIK_T, ARM_JIK)
#include "legs.h"
REGISTER_CONTROLLER(control::LEG_AIK_T, LEG_AIK)
#include "pelvis.h"
REGISTER_CONTROLLER(control::BODY_XYZRPY_FIX_LEGS_T, BODY_XYZRPY_FIX_LEGS)
REGISTER_CONTROLLER(control::BODY_XZP_FIX_LEGS_T, BODY_XZP_FIX_LEGS)
REGISTER_CONTROLLER(control::BODY_ZY_FIX_LEGS_T, BODY_ZY_FIX_LEGS)
#include "joints.h"
REGISTER_CONTROLLER(control::JOINT_T, JOINT)
REGISTER_CONTROLLER(control::GRASP_T, GRASP)


//############################################################
//### Implementation
namespace control {
    
    //############################################################
    //### Global variables and functions
    //############################################################
    std::vector<control_factory_t*> *control_factory_t::_factories = 0;

    const control_factory_t* get_factory(std::string name)
    {
        const std::vector<control_factory_t*>& f = factories();
        for(int i=0; i < f.size(); i++) {
            std::cout << "Factory " << f[i]->name() << std::endl;
        }
        
        BOOST_FOREACH(control_factory_t* factory, control_factory_t::factory_vector()) {
            if(name == factory->name())
                return factory;
        }
        std::cerr << "Error: Factory " << name << " not found." << std::endl;
        return NULL;
    }
    
    const std::vector<control_factory_t*>& factories() {
        return control_factory_t::factory_vector();
    }

    const std::vector<control_factory_t*>& control_factory_t::factory_vector()
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
            _factories = new std::vector<control_factory_t*>();
        }
        _factories->push_back(this);
    }

    control_factory_t::~control_factory_t() {}
}
