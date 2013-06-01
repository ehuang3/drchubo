/********************************************************/
/*    Author: Matt Zucker <mzucker1@swarthmore.edu>     */
/* Copyright (c) 2012 Matt Zucker. All rights reserved. */
/********************************************************/
#ifndef _TINYDOM_H_
#define _TINYDOM_H_

#include <string>
#include <list>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <stdio.h>

//////////////////////////////////////////////////////////////////////

class TinyDomElement;

class TinyDomNode {
public:

  enum NodeType {
    TypeElement,
    TypeCharData,
  };

  virtual ~TinyDomNode();

  const TinyDomElement* parent() const;
  TinyDomElement* parent();

  void reparent(TinyDomElement* parent);

  virtual NodeType type() const =0;



protected:

  TinyDomNode(TinyDomElement* parent);

private:

  TinyDomElement* _parent;
  friend class TinyDomElement;

};

//////////////////////////////////////////////////////////////////////

class TinyDomAttribute {
public:
  std::string key;
  std::string data;
};

//////////////////////////////////////////////////////////////////////
  
class TinyDomElement: public TinyDomNode {
public:

  typedef std::list<TinyDomAttribute> AttributeList;
  typedef std::list<TinyDomNode*> ChildList;


private:

  AttributeList _attributes;
  ChildList _children;

  std::string _name;

public:

  


  TinyDomElement(const std::string& name, TinyDomElement* parent=0);
  TinyDomElement(const char* name, TinyDomElement* parent=0);
  TinyDomElement(TinyDomElement* parent=0);

  virtual ~TinyDomElement();

  const std::string& name() const;
  void setName(const std::string& str);

  virtual NodeType type() const;

  std::string attribute(const std::string& key, 
			const std::string& defaultValue=std::string()) const;

  const AttributeList& attributes() const;

  bool haveAttribute(const std::string& key) const;

  void clearAttribute(const std::string& key);
  
  void setAttribute(const std::string& key, const std::string& data);

  template <class Tval> void setAttributeValue(const std::string& key, Tval value) {
    std::ostringstream ostr;
    ostr << value;
    setAttribute(key, ostr.str());
  }


  template <class Tval> Tval attributeValue(const std::string& key, Tval defaultValue) const {
    if (!haveAttribute(key)) { return defaultValue; }
    std::istringstream istr(attribute(key));
    Tval rval = defaultValue;
    if (!(istr >> rval) || (istr.peek() != EOF)) {
		throw std::runtime_error("error parsing attribute value: "+key);
    }
    return rval;
  }

  std::string attributeValue(const std::string& key, const std::string& defaultValue) const {
    return attribute(key, defaultValue);
  }
					    

  const ChildList& children() const;

  void addChild(TinyDomNode* child, 
		const TinyDomNode* before=0);

  void removeChild(TinyDomNode* child);

};

//////////////////////////////////////////////////////////////////////

class TinyDomCharacterData: public TinyDomNode {
public:

  TinyDomCharacterData(TinyDomElement* parent);
  TinyDomCharacterData(const std::string& value, TinyDomElement* parent=0);
  TinyDomCharacterData(const char* value, TinyDomElement* parent=0);

  virtual ~TinyDomCharacterData();

  virtual NodeType type() const;
  
  std::string value;

  std::string trimmedValue() const;
  
};

//////////////////////////////////////////////////////////////////////

class TinyDom {
public:

  static std::string escape(const std::string& str);
  static std::string unescape(const std::string& str);

  static TinyDomElement* parse(std::istream& istr);

  static void unparse(const TinyDomElement* element, 
		      std::ostream& ostr);

};

  
#endif
