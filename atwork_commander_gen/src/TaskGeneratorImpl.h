#pragma once


#include <iostream>
#include <unordered_map>
#include <regex>

using namespace std;

namespace atwork_commander {

class Object;
class Table;

using ObjectPtr = Object*;
using TablePtr = Table*;

enum class Orientation : unsigned int {
  FREE,       ///< Orientation to be chosen by the Team
  VERTICAL,   ///< Vertical Orientation
  HORIZONTAL  ///< Horizontal Orientation
};

enum class Type : unsigned int {
  UNKNOWN,        ///< unknown object( invalid )
  OBJECT,         ///< Plain Object without special properties
  COLORED_OBJECT, ///< Colored Object
  CAVITY,         ///< PPT Cavity
  CONTAINER       ///< Container( currently RED or BLUE )
};

struct ObjectBase {
  static Type extractType(const string& typeName) {
    size_t len = typeName.length();
    char preLast = typeName[ len - 2 ];
    if( preLast == '_' ) {
      char last = typeName[ len - 1 ];
      if (  last == 'H' || last == 'V' )
        return Type::CAVITY;
      if (  last == 'G' || last == 'B' )
        return Type::COLORED_OBJECT;
    }
    if ( regex_match( typeName, regex( "CONTAINER_.*" ) ) )
      return Type::CONTAINER;
    return Type::OBJECT;
  }

  string extractForm(const string& typeName) const {
    switch ( type ) {
      case ( Type::CAVITY):
      case ( Type::COLORED_OBJECT): return typeName.substr( 0, typeName.length() - 2 );
      case ( Type::CONTAINER ): return "DEFAULT";
      default: return typeName;
    }
  }

  string extractColor(const string& typeName) const {
    switch ( type ) {
      case ( Type::COLORED_OBJECT): return typeName.substr( typeName.length() - 1 );
      case ( Type::CONTAINER ): return typeName.substr( strlen("CONTAINER_") );
      default: return "DEFAULT";
    }
  }

  Orientation extractOrientation(const string& typeName) const {
    if ( type ==  Type::CAVITY)
      switch ( typeName[ typeName.length() - 1 ] ) {
        case ( 'H' ): return Orientation::HORIZONTAL;
        case ( 'V' ): return Orientation::VERTICAL;
      };
    return Orientation::FREE;
  }

  Type type = Type::UNKNOWN;
  string form = "";
  string color = "";
  Orientation orientation = Orientation::FREE;
  ObjectBase() = default;
  ObjectBase(const string typeName)
    : type( extractType( typeName ) ), form( extractForm( typeName ) ),
      color( extractColor( typeName ) ),
      orientation( extractOrientation( typeName ) )
  {}
};

struct ObjectType : public ObjectBase {
  unsigned int count = 0;
  ObjectType( const string& typeName, unsigned int count )
    : ObjectBase( typeName ), count(count)
  {}
  operator bool() const { return count; }
  ObjectType& operator--() {
    count--;
    return *this;
  }
  ObjectType operator--(int) {
    ObjectType temp(*this);
    count--;
    return temp;
  }
};

struct Object : public ObjectBase {
  static unsigned int globalID;
  unsigned int id=0;
  TablePtr source = nullptr;
  TablePtr destination = nullptr;
  ObjectPtr container = nullptr;
  Object() = default;
  Object(ObjectType& type): ObjectBase(type), id(globalID++) { type--; }
  Object(ObjectType&& type): ObjectBase(type), id(globalID++) {}
  static void reset() { globalID = 0; }
};

unsigned int Object::globalID = 1;

struct Table {
  std::string name ="";
  std::string type ="";
  Table() = default;
  Table(const std::string& name, const std::string& type)
    : name(name), type(type) {}
};

}

ostream& operator<<(ostream& os, const atwork_commander::Type type) {
  switch ( type ) {
    case ( atwork_commander::Type::CAVITY ):         return os << "Cavity";
    case ( atwork_commander::Type::CONTAINER ):      return os << "Container";
    case ( atwork_commander::Type::COLORED_OBJECT ): return os << "Colored Object";
    case ( atwork_commander::Type::OBJECT ):         return os << "Plain Object";
    default:                                          return os << "UNKNOWN";
  }
}

ostream& operator<<(ostream& os, const atwork_commander::Orientation o) {
  switch ( o ) {
    case( atwork_commander::Orientation::VERTICAL )  : return os << "V";
    case( atwork_commander::Orientation::HORIZONTAL ): return os << "H";
    case( atwork_commander::Orientation::FREE )      : return os << "FREE";
    default                                           : return os << "UNKNOWN";
  }
}

ostream& operator<<(ostream& os, const atwork_commander::ObjectBase& type) {
  switch ( type.type ) {
    case ( atwork_commander::Type::CAVITY ):         return os << type.form << "_" << type.orientation;
    case ( atwork_commander::Type::CONTAINER ):      return os << "CONTAINER_"     << type.color;
    case ( atwork_commander::Type::COLORED_OBJECT ): return os << type.form << "_" << type.color;
    case ( atwork_commander::Type::OBJECT ):         return os << type.form;
    default:                                           return os << "UNKNOWN OBJECT TYPE";
  }
}

ostream& operator<<(ostream& os, const atwork_commander::ObjectType& type) {
  return os << ((atwork_commander::ObjectBase)type) << "(" << type.count << ")";
}


ostream& operator<<(ostream& os, const atwork_commander::Table& t) {
  return os << "Table " << t.name << "(" << t.type << "):";
}

ostream& operator<<(ostream& os, const atwork_commander::Object& o) {
  os << "Object " << atwork_commander::ObjectBase(o) << "(" << o.id << "):";
  if ( o.source )      os << " Src: " << o.source->name      << "(" << o.source->type      << ")";
  if ( o.destination ) os << " Dst: " << o.destination->name << "(" << o.destination->type << ")";
  if ( o.container )   os << " Cont: " << o.container->type   << "(" << o.container->id     << ")";
  return os;
}

ostream& operator<<(ostream& os, const vector<atwork_commander::Object>& v) {
  for (size_t i=0; i<v.size(); i++)
    os << v[i] << (i+1!=v.size()?"\n":"");
  return os;
}

template<typename T>
ostream& operator<<(ostream& os, const vector<T>& v) {
  for (size_t i=0; i<v.size(); i++)
    os << v[i] << (i+1!=v.size()?" ":"");
  return os;
}

template<typename K, typename V>
ostream& operator<<(ostream& os, const unordered_multimap<K, V>& m) {
  for (const auto& item: m)
    os << item.first << " = " << item.second << endl;
  return os;
}

template<typename T, size_t n>
ostream& operator<<(ostream& os, const array<T,n>& vec)
{
  for(size_t i=0; i<vec.size(); i++)
		os << vec[i] << (i==vec.size()-1?"":" ");
  return os << endl;
}

template<typename T, size_t n>
ostream& operator<<(ostream& os, const vector<array<T,n>>& vec)
{
  for(size_t i=0; i<vec.size(); i++)
		os << vec[i];
  return os << endl;
}

template<typename T>
ostream& operator<<(ostream& os, const vector<vector<T>>& vec)
{
  for(size_t i=0; i<vec.size(); i++)
		os << vec[i];
  return os << endl;
}

template<typename T1, typename T2>
ostream& operator<<(ostream& os, const map<T1, T2>& m)
{
  for(const typename map<T1, T2>::value_type& v: m)
		os << v.first << ": " << v.second << endl;
  return os;
}
