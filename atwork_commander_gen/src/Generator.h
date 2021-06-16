#pragma once

#include <atwork_commander_gen/GeneratorPluginInterface.h>

#include <iostream>
#include <unordered_map>
#include <regex>

namespace atwork_commander {
namespace task_generator {

using Task = GeneratorPluginInterface::Task;

struct Object;
struct ObjectType;
struct Table;

using ObjectPtr = Object*;
using ObjectTypePtr = ObjectType*;
using TablePtr = Table*;

enum class Orientation : unsigned int {
  HORIZONTAL,///< Horizontal Orientation
  VERTICAL,  ///< Vertical Orientation
  FREE       ///< Orientation to be chosen by the Team
};

enum class Type : unsigned int {
  UNKNOWN,        ///< unknown object( invalid )
  OBJECT,         ///< Plain Object without special properties
  COLORED_OBJECT, ///< Colored Object
  CAVITY,         ///< PPT Cavity
  CONTAINER       ///< Container( currently RED or BLUE )
};

struct ObjectBase {
  static Type extractType(const std::string& typeName) {
    size_t len = typeName.length();
    char preLast = typeName[ len - 2 ];
    if( preLast == '_' ) {
      char last = typeName[ len - 1 ];
      if (  last == 'H' || last == 'V' )
        return Type::CAVITY;
      if (  last == 'G' || last == 'B' )
        return Type::COLORED_OBJECT;
    }
    if ( std::regex_match( typeName, std::regex( "CONTAINER_.*" ) ) )
      return Type::CONTAINER;
    return Type::OBJECT;
  }

  std::string extractForm(const std::string& typeName) const {
    switch ( type ) {
      case ( Type::CAVITY):
      case ( Type::COLORED_OBJECT): return typeName.substr( 0, typeName.length() - 2 );
      case ( Type::CONTAINER ): return "DEFAULT";
      default: return typeName;
    }
  }

  std::string extractColor(const std::string& typeName) const {
    switch ( type ) {
      case ( Type::COLORED_OBJECT): return typeName.substr( typeName.length() - 1 );
      case ( Type::CONTAINER ): return typeName.substr( strlen("CONTAINER_") );
      default: return "DEFAULT";
    }
  }

  Orientation extractOrientation(const std::string& typeName) const {
    if ( type ==  Type::CAVITY)
      switch ( typeName[ typeName.length() - 1 ] ) {
        case ( 'H' ): return Orientation::HORIZONTAL;
        case ( 'V' ): return Orientation::VERTICAL;
      };
    return Orientation::FREE;
  }

  Type type = Type::UNKNOWN;
  std::string form = "";
  std::string color = "";
  Orientation orientation = Orientation::FREE;
  ObjectBase() = default;
  ObjectBase(const std::string typeName)
    : type( extractType( typeName ) ), form( extractForm( typeName ) ),
      color( extractColor( typeName ) ),
      orientation( extractOrientation( typeName ) )
  {}
  bool operator==(const ObjectBase& b) const {
    return type == b.type && form == b.form && color == b.color;
  }
  bool operator<(const ObjectBase& b) const {
    if (type < b.type)
      return true;
    if (form < b.form)
      return true;
    if (color < b.color)
      return true;
    return false;
  }
};

struct ObjectType : public ObjectBase {
  unsigned int count = 0;
  ObjectType( const std::string& typeName, unsigned int count )
    : ObjectBase( typeName ), count(count)
  {}
  operator bool() const { return count; }
  ObjectType& operator--() {
    count = std::max(1u, count)-1;
    return *this;
  }
  ObjectType operator--(int) {
    ObjectType temp(*this);
    count = std::max(1u, count)-1;
    return temp;
  }
  using ObjectBase::operator<;
  using ObjectBase::operator==;
};

struct Object : public ObjectBase {
  static unsigned int globalID;
  unsigned int id=0;
  TablePtr source = nullptr;
  TablePtr destination = nullptr;
  ObjectTypePtr container = nullptr;
  Object() = default;
  Object(ObjectType& type): ObjectBase(type), id(globalID++) { type--; }
  Object(ObjectType&& type): ObjectBase(type), id(globalID++) {}
  static void reset() { globalID = 1; }
};


struct Table {
  std::string name ="";
  std::string type ="";
  Table() = default;
  Table(const std::string& name, const std::string& type)
    : name(name), type(type) {}
};

}
}

inline std::ostream& operator<<(std::ostream& os, const atwork_commander::task_generator::Type type) {
  switch ( type ) {
    case ( atwork_commander::task_generator::Type::CAVITY ):         return os << "Cavity";
    case ( atwork_commander::task_generator::Type::CONTAINER ):      return os << "Container";
    case ( atwork_commander::task_generator::Type::COLORED_OBJECT ): return os << "Colored Object";
    case ( atwork_commander::task_generator::Type::OBJECT ):         return os << "Plain Object";
    default:                                          return os << "UNKNOWN";
  }
}

inline std::ostream& operator<<(std::ostream& os, const atwork_commander::task_generator::Orientation o) {
  switch ( o ) {
    case( atwork_commander::task_generator::Orientation::VERTICAL )  : return os << "V";
    case( atwork_commander::task_generator::Orientation::HORIZONTAL ): return os << "H";
    case( atwork_commander::task_generator::Orientation::FREE )      : return os << "FREE";
    default                                           : return os << "UNKNOWN";
  }
}

inline std::ostream& operator<<(std::ostream& os, const atwork_commander::task_generator::ObjectBase& type) {
  switch ( type.type ) {
    case ( atwork_commander::task_generator::Type::CAVITY ):         return os << type.form << "_" << type.orientation;
    case ( atwork_commander::task_generator::Type::CONTAINER ):      return os << "CONTAINER_"     << type.color;
    case ( atwork_commander::task_generator::Type::COLORED_OBJECT ): return os << type.form << "_" << type.color;
    case ( atwork_commander::task_generator::Type::OBJECT ):         return os << type.form;
    default:                                           return os << "UNKNOWN OBJECT TYPE";
  }
}

inline std::ostream& operator<<(std::ostream& os, const atwork_commander::task_generator::ObjectType& type) {
  return os << ((atwork_commander::task_generator::ObjectBase)type) << "(" << type.count << ")";
}


inline std::ostream& operator<<(std::ostream& os, const atwork_commander::task_generator::Table& t) {
  return os << "Table " << t.name << "(" << t.type << "):";
}

inline std::ostream& operator<<(std::ostream& os, const atwork_commander::task_generator::Object& o) {
  os << "Object " << atwork_commander::task_generator::ObjectBase(o) << "(" << o.id << "):";
  if ( o.source )      os << " Src: " << o.source->name      << "(" << o.source->type      << ")";
  if ( o.destination ) os << " Dst: " << o.destination->name << "(" << o.destination->type << ")";
  if ( o.container )   os << " Cont: " << o.container->type;
  return os;
}

inline std::ostream& operator<<(std::ostream& os, const std::vector<atwork_commander::task_generator::Object>& v) {
  for (size_t i=0; i<v.size(); i++)
    os << v[i] << (i+1!=v.size()?"\n":"");
  return os;
}

template<typename T>
std::ostream& operator<<(std::ostream& os, const std::vector<T>& v) {
  for (size_t i=0; i<v.size(); i++)
    os << v[i] << (i+1!=v.size()?" ":"");
  return os;
}

template<typename K, typename V>
std::ostream& operator<<(std::ostream& os, const std::unordered_multimap<K, V>& m) {
  for (const auto& item: m)
    os << item.first << " = " << item.second << std::endl;
  return os;
}

template<typename T, size_t n>
std::ostream& operator<<(std::ostream& os, const std::array<T,n>& vec)
{
  for(size_t i=0; i<vec.size(); i++) os << vec[i] << (i==vec.size()-1?"":" ");
  return os << std::endl;
}

template<typename T, size_t n>
std::ostream& operator<<(std::ostream& os, const std::vector<std::array<T,n>>& vec)
{
  for(size_t i=0; i<vec.size(); i++) os << vec[i];
  return os << std::endl;
}

template<typename T>
std::ostream& operator<<(std::ostream& os, const std::vector<std::vector<T>>& vec)
{
  for(size_t i=0; i<vec.size(); i++) os << vec[i];
  return os << std::endl;
}

template<typename T>
std::ostream& operator<<(std::ostream& os, const std::set<T>& s)
{
  os << "{ ";
  for(const auto& obj: s)
    os << obj << " ";
  return os << "}";
}


template<typename T1, typename T2>
std::ostream& operator<<(std::ostream& os, const std::map<T1, T2>& m)
{
  for(const typename std::map<T1, T2>::value_type& v: m)
		os << v.first << ": " << v.second << std::endl;
  return os;
}

