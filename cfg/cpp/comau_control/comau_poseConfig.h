//#line 2 "/opt/ros/noetic/share/dynamic_reconfigure/templates/ConfigType.h.template"
// *********************************************************
//
// File autogenerated for the comau_control package
// by the dynamic_reconfigure package.
// Please do not edit.
//
// ********************************************************/

#ifndef __comau_control__COMAU_POSECONFIG_H__
#define __comau_control__COMAU_POSECONFIG_H__

#if __cplusplus >= 201103L
#define DYNAMIC_RECONFIGURE_FINAL final
#else
#define DYNAMIC_RECONFIGURE_FINAL
#endif

#include <dynamic_reconfigure/config_tools.h>
#include <limits>
#include <ros/node_handle.h>
#include <dynamic_reconfigure/ConfigDescription.h>
#include <dynamic_reconfigure/ParamDescription.h>
#include <dynamic_reconfigure/Group.h>
#include <dynamic_reconfigure/config_init_mutex.h>
#include <boost/any.hpp>

namespace comau_control
{
  class comau_poseConfigStatics;

  class comau_poseConfig
  {
  public:
    class AbstractParamDescription : public dynamic_reconfigure::ParamDescription
    {
    public:
      AbstractParamDescription(std::string n, std::string t, uint32_t l,
          std::string d, std::string e)
      {
        name = n;
        type = t;
        level = l;
        description = d;
        edit_method = e;
      }
      virtual ~AbstractParamDescription() = default;

      virtual void clamp(comau_poseConfig &config, const comau_poseConfig &max, const comau_poseConfig &min) const = 0;
      virtual void calcLevel(uint32_t &level, const comau_poseConfig &config1, const comau_poseConfig &config2) const = 0;
      virtual void fromServer(const ros::NodeHandle &nh, comau_poseConfig &config) const = 0;
      virtual void toServer(const ros::NodeHandle &nh, const comau_poseConfig &config) const = 0;
      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, comau_poseConfig &config) const = 0;
      virtual void toMessage(dynamic_reconfigure::Config &msg, const comau_poseConfig &config) const = 0;
      virtual void getValue(const comau_poseConfig &config, boost::any &val) const = 0;
    };

    typedef boost::shared_ptr<AbstractParamDescription> AbstractParamDescriptionPtr;
    typedef boost::shared_ptr<const AbstractParamDescription> AbstractParamDescriptionConstPtr;

    // Final keyword added to class because it has virtual methods and inherits
    // from a class with a non-virtual destructor.
    template <class T>
    class ParamDescription DYNAMIC_RECONFIGURE_FINAL : public AbstractParamDescription
    {
    public:
      ParamDescription(std::string a_name, std::string a_type, uint32_t a_level,
          std::string a_description, std::string a_edit_method, T comau_poseConfig::* a_f) :
        AbstractParamDescription(a_name, a_type, a_level, a_description, a_edit_method),
        field(a_f)
      {}

      T comau_poseConfig::* field;

      virtual void clamp(comau_poseConfig &config, const comau_poseConfig &max, const comau_poseConfig &min) const override
      {
        if (config.*field > max.*field)
          config.*field = max.*field;

        if (config.*field < min.*field)
          config.*field = min.*field;
      }

      virtual void calcLevel(uint32_t &comb_level, const comau_poseConfig &config1, const comau_poseConfig &config2) const override
      {
        if (config1.*field != config2.*field)
          comb_level |= level;
      }

      virtual void fromServer(const ros::NodeHandle &nh, comau_poseConfig &config) const override
      {
        nh.getParam(name, config.*field);
      }

      virtual void toServer(const ros::NodeHandle &nh, const comau_poseConfig &config) const override
      {
        nh.setParam(name, config.*field);
      }

      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, comau_poseConfig &config) const override
      {
        return dynamic_reconfigure::ConfigTools::getParameter(msg, name, config.*field);
      }

      virtual void toMessage(dynamic_reconfigure::Config &msg, const comau_poseConfig &config) const override
      {
        dynamic_reconfigure::ConfigTools::appendParameter(msg, name, config.*field);
      }

      virtual void getValue(const comau_poseConfig &config, boost::any &val) const override
      {
        val = config.*field;
      }
    };

    class AbstractGroupDescription : public dynamic_reconfigure::Group
    {
      public:
      AbstractGroupDescription(std::string n, std::string t, int p, int i, bool s)
      {
        name = n;
        type = t;
        parent = p;
        state = s;
        id = i;
      }

      virtual ~AbstractGroupDescription() = default;

      std::vector<AbstractParamDescriptionConstPtr> abstract_parameters;
      bool state;

      virtual void toMessage(dynamic_reconfigure::Config &msg, const boost::any &config) const = 0;
      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, boost::any &config) const =0;
      virtual void updateParams(boost::any &cfg, comau_poseConfig &top) const= 0;
      virtual void setInitialState(boost::any &cfg) const = 0;


      void convertParams()
      {
        for(std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = abstract_parameters.begin(); i != abstract_parameters.end(); ++i)
        {
          parameters.push_back(dynamic_reconfigure::ParamDescription(**i));
        }
      }
    };

    typedef boost::shared_ptr<AbstractGroupDescription> AbstractGroupDescriptionPtr;
    typedef boost::shared_ptr<const AbstractGroupDescription> AbstractGroupDescriptionConstPtr;

    // Final keyword added to class because it has virtual methods and inherits
    // from a class with a non-virtual destructor.
    template<class T, class PT>
    class GroupDescription DYNAMIC_RECONFIGURE_FINAL : public AbstractGroupDescription
    {
    public:
      GroupDescription(std::string a_name, std::string a_type, int a_parent, int a_id, bool a_s, T PT::* a_f) : AbstractGroupDescription(a_name, a_type, a_parent, a_id, a_s), field(a_f)
      {
      }

      GroupDescription(const GroupDescription<T, PT>& g): AbstractGroupDescription(g.name, g.type, g.parent, g.id, g.state), field(g.field), groups(g.groups)
      {
        parameters = g.parameters;
        abstract_parameters = g.abstract_parameters;
      }

      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, boost::any &cfg) const override
      {
        PT* config = boost::any_cast<PT*>(cfg);
        if(!dynamic_reconfigure::ConfigTools::getGroupState(msg, name, (*config).*field))
          return false;

        for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); ++i)
        {
          boost::any n = &((*config).*field);
          if(!(*i)->fromMessage(msg, n))
            return false;
        }

        return true;
      }

      virtual void setInitialState(boost::any &cfg) const override
      {
        PT* config = boost::any_cast<PT*>(cfg);
        T* group = &((*config).*field);
        group->state = state;

        for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); ++i)
        {
          boost::any n = boost::any(&((*config).*field));
          (*i)->setInitialState(n);
        }

      }

      virtual void updateParams(boost::any &cfg, comau_poseConfig &top) const override
      {
        PT* config = boost::any_cast<PT*>(cfg);

        T* f = &((*config).*field);
        f->setParams(top, abstract_parameters);

        for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); ++i)
        {
          boost::any n = &((*config).*field);
          (*i)->updateParams(n, top);
        }
      }

      virtual void toMessage(dynamic_reconfigure::Config &msg, const boost::any &cfg) const override
      {
        const PT config = boost::any_cast<PT>(cfg);
        dynamic_reconfigure::ConfigTools::appendGroup<T>(msg, name, id, parent, config.*field);

        for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); ++i)
        {
          (*i)->toMessage(msg, config.*field);
        }
      }

      T PT::* field;
      std::vector<comau_poseConfig::AbstractGroupDescriptionConstPtr> groups;
    };

class DEFAULT
{
  public:
    DEFAULT()
    {
      state = true;
      name = "Default";
    }

    void setParams(comau_poseConfig &config, const std::vector<AbstractParamDescriptionConstPtr> params)
    {
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator _i = params.begin(); _i != params.end(); ++_i)
      {
        boost::any val;
        (*_i)->getValue(config, val);

        if("end_effector_x"==(*_i)->name){end_effector_x = boost::any_cast<double>(val);}
        if("end_effector_y"==(*_i)->name){end_effector_y = boost::any_cast<double>(val);}
        if("end_effector_z"==(*_i)->name){end_effector_z = boost::any_cast<double>(val);}
        if("end_effector_roll"==(*_i)->name){end_effector_roll = boost::any_cast<double>(val);}
        if("end_effector_pitch"==(*_i)->name){end_effector_pitch = boost::any_cast<double>(val);}
        if("end_effector_yaw"==(*_i)->name){end_effector_yaw = boost::any_cast<double>(val);}
        if("activate_motion"==(*_i)->name){activate_motion = boost::any_cast<bool>(val);}
      }
    }

    double end_effector_x;
double end_effector_y;
double end_effector_z;
double end_effector_roll;
double end_effector_pitch;
double end_effector_yaw;
bool activate_motion;

    bool state;
    std::string name;

    
}groups;



//#line 273 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      double end_effector_x;
//#line 273 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      double end_effector_y;
//#line 273 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      double end_effector_z;
//#line 273 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      double end_effector_roll;
//#line 273 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      double end_effector_pitch;
//#line 273 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      double end_effector_yaw;
//#line 273 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      bool activate_motion;
//#line 231 "/opt/ros/noetic/share/dynamic_reconfigure/templates/ConfigType.h.template"

    bool __fromMessage__(dynamic_reconfigure::Config &msg)
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__ = __getGroupDescriptions__();

      int count = 0;
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        if ((*i)->fromMessage(msg, *this))
          count++;

      for (std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin(); i != __group_descriptions__.end(); i ++)
      {
        if ((*i)->id == 0)
        {
          boost::any n = boost::any(this);
          (*i)->updateParams(n, *this);
          (*i)->fromMessage(msg, n);
        }
      }

      if (count != dynamic_reconfigure::ConfigTools::size(msg))
      {
        ROS_ERROR("comau_poseConfig::__fromMessage__ called with an unexpected parameter.");
        ROS_ERROR("Booleans:");
        for (unsigned int i = 0; i < msg.bools.size(); i++)
          ROS_ERROR("  %s", msg.bools[i].name.c_str());
        ROS_ERROR("Integers:");
        for (unsigned int i = 0; i < msg.ints.size(); i++)
          ROS_ERROR("  %s", msg.ints[i].name.c_str());
        ROS_ERROR("Doubles:");
        for (unsigned int i = 0; i < msg.doubles.size(); i++)
          ROS_ERROR("  %s", msg.doubles[i].name.c_str());
        ROS_ERROR("Strings:");
        for (unsigned int i = 0; i < msg.strs.size(); i++)
          ROS_ERROR("  %s", msg.strs[i].name.c_str());
        // @todo Check that there are no duplicates. Make this error more
        // explicit.
        return false;
      }
      return true;
    }

    // This version of __toMessage__ is used during initialization of
    // statics when __getParamDescriptions__ can't be called yet.
    void __toMessage__(dynamic_reconfigure::Config &msg, const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__, const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__) const
    {
      dynamic_reconfigure::ConfigTools::clear(msg);
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->toMessage(msg, *this);

      for (std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin(); i != __group_descriptions__.end(); ++i)
      {
        if((*i)->id == 0)
        {
          (*i)->toMessage(msg, *this);
        }
      }
    }

    void __toMessage__(dynamic_reconfigure::Config &msg) const
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__ = __getGroupDescriptions__();
      __toMessage__(msg, __param_descriptions__, __group_descriptions__);
    }

    void __toServer__(const ros::NodeHandle &nh) const
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->toServer(nh, *this);
    }

    void __fromServer__(const ros::NodeHandle &nh)
    {
      static bool setup=false;

      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->fromServer(nh, *this);

      const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__ = __getGroupDescriptions__();
      for (std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin(); i != __group_descriptions__.end(); i++){
        if (!setup && (*i)->id == 0) {
          setup = true;
          boost::any n = boost::any(this);
          (*i)->setInitialState(n);
        }
      }
    }

    void __clamp__()
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      const comau_poseConfig &__max__ = __getMax__();
      const comau_poseConfig &__min__ = __getMin__();
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->clamp(*this, __max__, __min__);
    }

    uint32_t __level__(const comau_poseConfig &config) const
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      uint32_t level = 0;
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->calcLevel(level, config, *this);
      return level;
    }

    static const dynamic_reconfigure::ConfigDescription &__getDescriptionMessage__();
    static const comau_poseConfig &__getDefault__();
    static const comau_poseConfig &__getMax__();
    static const comau_poseConfig &__getMin__();
    static const std::vector<AbstractParamDescriptionConstPtr> &__getParamDescriptions__();
    static const std::vector<AbstractGroupDescriptionConstPtr> &__getGroupDescriptions__();

  private:
    static const comau_poseConfigStatics *__get_statics__();
  };

  template <> // Max and min are ignored for strings.
  inline void comau_poseConfig::ParamDescription<std::string>::clamp(comau_poseConfig &config, const comau_poseConfig &max, const comau_poseConfig &min) const
  {
    (void) config;
    (void) min;
    (void) max;
    return;
  }

  class comau_poseConfigStatics
  {
    friend class comau_poseConfig;

    comau_poseConfigStatics()
    {
comau_poseConfig::GroupDescription<comau_poseConfig::DEFAULT, comau_poseConfig> Default("Default", "", 0, 0, true, &comau_poseConfig::groups);
//#line 273 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __min__.end_effector_x = 0.0;
//#line 273 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __max__.end_effector_x = 1.4;
//#line 273 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __default__.end_effector_x = 0.8;
//#line 273 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      Default.abstract_parameters.push_back(comau_poseConfig::AbstractParamDescriptionConstPtr(new comau_poseConfig::ParamDescription<double>("end_effector_x", "double", 0, "The position value for x", "", &comau_poseConfig::end_effector_x)));
//#line 273 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __param_descriptions__.push_back(comau_poseConfig::AbstractParamDescriptionConstPtr(new comau_poseConfig::ParamDescription<double>("end_effector_x", "double", 0, "The position value for x", "", &comau_poseConfig::end_effector_x)));
//#line 273 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __min__.end_effector_y = -2.0;
//#line 273 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __max__.end_effector_y = 0.5;
//#line 273 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __default__.end_effector_y = 0.0;
//#line 273 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      Default.abstract_parameters.push_back(comau_poseConfig::AbstractParamDescriptionConstPtr(new comau_poseConfig::ParamDescription<double>("end_effector_y", "double", 0, "The position value for y", "", &comau_poseConfig::end_effector_y)));
//#line 273 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __param_descriptions__.push_back(comau_poseConfig::AbstractParamDescriptionConstPtr(new comau_poseConfig::ParamDescription<double>("end_effector_y", "double", 0, "The position value for y", "", &comau_poseConfig::end_effector_y)));
//#line 273 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __min__.end_effector_z = 0.4;
//#line 273 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __max__.end_effector_z = 1.6;
//#line 273 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __default__.end_effector_z = 1.0;
//#line 273 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      Default.abstract_parameters.push_back(comau_poseConfig::AbstractParamDescriptionConstPtr(new comau_poseConfig::ParamDescription<double>("end_effector_z", "double", 0, "The position value for z", "", &comau_poseConfig::end_effector_z)));
//#line 273 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __param_descriptions__.push_back(comau_poseConfig::AbstractParamDescriptionConstPtr(new comau_poseConfig::ParamDescription<double>("end_effector_z", "double", 0, "The position value for z", "", &comau_poseConfig::end_effector_z)));
//#line 273 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __min__.end_effector_roll = -1.57;
//#line 273 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __max__.end_effector_roll = 1.57;
//#line 273 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __default__.end_effector_roll = 0.0;
//#line 273 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      Default.abstract_parameters.push_back(comau_poseConfig::AbstractParamDescriptionConstPtr(new comau_poseConfig::ParamDescription<double>("end_effector_roll", "double", 0, "The position value for roll", "", &comau_poseConfig::end_effector_roll)));
//#line 273 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __param_descriptions__.push_back(comau_poseConfig::AbstractParamDescriptionConstPtr(new comau_poseConfig::ParamDescription<double>("end_effector_roll", "double", 0, "The position value for roll", "", &comau_poseConfig::end_effector_roll)));
//#line 273 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __min__.end_effector_pitch = -1.57;
//#line 273 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __max__.end_effector_pitch = 1.57;
//#line 273 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __default__.end_effector_pitch = 0.0;
//#line 273 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      Default.abstract_parameters.push_back(comau_poseConfig::AbstractParamDescriptionConstPtr(new comau_poseConfig::ParamDescription<double>("end_effector_pitch", "double", 0, "The position value for pitch", "", &comau_poseConfig::end_effector_pitch)));
//#line 273 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __param_descriptions__.push_back(comau_poseConfig::AbstractParamDescriptionConstPtr(new comau_poseConfig::ParamDescription<double>("end_effector_pitch", "double", 0, "The position value for pitch", "", &comau_poseConfig::end_effector_pitch)));
//#line 273 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __min__.end_effector_yaw = -6.28;
//#line 273 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __max__.end_effector_yaw = 6.28;
//#line 273 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __default__.end_effector_yaw = 0.0;
//#line 273 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      Default.abstract_parameters.push_back(comau_poseConfig::AbstractParamDescriptionConstPtr(new comau_poseConfig::ParamDescription<double>("end_effector_yaw", "double", 0, "The position value for yaw", "", &comau_poseConfig::end_effector_yaw)));
//#line 273 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __param_descriptions__.push_back(comau_poseConfig::AbstractParamDescriptionConstPtr(new comau_poseConfig::ParamDescription<double>("end_effector_yaw", "double", 0, "The position value for yaw", "", &comau_poseConfig::end_effector_yaw)));
//#line 273 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __min__.activate_motion = 0;
//#line 273 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __max__.activate_motion = 1;
//#line 273 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __default__.activate_motion = 0;
//#line 273 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      Default.abstract_parameters.push_back(comau_poseConfig::AbstractParamDescriptionConstPtr(new comau_poseConfig::ParamDescription<bool>("activate_motion", "bool", 0, "Immediately accept modifications", "", &comau_poseConfig::activate_motion)));
//#line 273 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __param_descriptions__.push_back(comau_poseConfig::AbstractParamDescriptionConstPtr(new comau_poseConfig::ParamDescription<bool>("activate_motion", "bool", 0, "Immediately accept modifications", "", &comau_poseConfig::activate_motion)));
//#line 245 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      Default.convertParams();
//#line 245 "/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __group_descriptions__.push_back(comau_poseConfig::AbstractGroupDescriptionConstPtr(new comau_poseConfig::GroupDescription<comau_poseConfig::DEFAULT, comau_poseConfig>(Default)));
//#line 369 "/opt/ros/noetic/share/dynamic_reconfigure/templates/ConfigType.h.template"

      for (std::vector<comau_poseConfig::AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin(); i != __group_descriptions__.end(); ++i)
      {
        __description_message__.groups.push_back(**i);
      }
      __max__.__toMessage__(__description_message__.max, __param_descriptions__, __group_descriptions__);
      __min__.__toMessage__(__description_message__.min, __param_descriptions__, __group_descriptions__);
      __default__.__toMessage__(__description_message__.dflt, __param_descriptions__, __group_descriptions__);
    }
    std::vector<comau_poseConfig::AbstractParamDescriptionConstPtr> __param_descriptions__;
    std::vector<comau_poseConfig::AbstractGroupDescriptionConstPtr> __group_descriptions__;
    comau_poseConfig __max__;
    comau_poseConfig __min__;
    comau_poseConfig __default__;
    dynamic_reconfigure::ConfigDescription __description_message__;

    static const comau_poseConfigStatics *get_instance()
    {
      // Split this off in a separate function because I know that
      // instance will get initialized the first time get_instance is
      // called, and I am guaranteeing that get_instance gets called at
      // most once.
      static comau_poseConfigStatics instance;
      return &instance;
    }
  };

  inline const dynamic_reconfigure::ConfigDescription &comau_poseConfig::__getDescriptionMessage__()
  {
    return __get_statics__()->__description_message__;
  }

  inline const comau_poseConfig &comau_poseConfig::__getDefault__()
  {
    return __get_statics__()->__default__;
  }

  inline const comau_poseConfig &comau_poseConfig::__getMax__()
  {
    return __get_statics__()->__max__;
  }

  inline const comau_poseConfig &comau_poseConfig::__getMin__()
  {
    return __get_statics__()->__min__;
  }

  inline const std::vector<comau_poseConfig::AbstractParamDescriptionConstPtr> &comau_poseConfig::__getParamDescriptions__()
  {
    return __get_statics__()->__param_descriptions__;
  }

  inline const std::vector<comau_poseConfig::AbstractGroupDescriptionConstPtr> &comau_poseConfig::__getGroupDescriptions__()
  {
    return __get_statics__()->__group_descriptions__;
  }

  inline const comau_poseConfigStatics *comau_poseConfig::__get_statics__()
  {
    const static comau_poseConfigStatics *statics;

    if (statics) // Common case
      return statics;

    boost::mutex::scoped_lock lock(dynamic_reconfigure::__init_mutex__);

    if (statics) // In case we lost a race.
      return statics;

    statics = comau_poseConfigStatics::get_instance();

    return statics;
  }


}

#undef DYNAMIC_RECONFIGURE_FINAL

#endif // __COMAU_POSERECONFIGURATOR_H__
