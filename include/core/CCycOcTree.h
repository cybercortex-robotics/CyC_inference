/*
 * 
 */

#ifndef OCTOMAP_CyC_OCTREE_H
#define OCTOMAP_CyC_OCTREE_H

#include <iostream>
#include <octomap/OcTreeNode.h>
#include <octomap/OccupancyOcTreeBase.h>

// forward declaraton for "friend"
class CCycOcTree;

// node definition
class CCycOcTreeNode : public octomap::OcTreeNode 
{    
public:
friend class CCycOcTree; // needs access to node children (inherited)

class Color
{
public:
    Color() : r(255), g(255), b(255) 
    {}
    Color(uint8_t _r, uint8_t _g, uint8_t _b) : r(_r), g(_g), b(_b) 
    {}

    inline bool operator== (const Color &other) const 
    {
        return (r==other.r && g==other.g && b==other.b);
    }
    inline bool operator!= (const Color &other) const 
    {
        return (r!=other.r || g!=other.g || b!=other.b);
    }
    uint8_t r, g, b;
};

public:
CCycOcTreeNode() : OcTreeNode() {}

CCycOcTreeNode(const CCycOcTreeNode& rhs) : OcTreeNode(rhs), color(rhs.color), object_class(rhs.object_class) {}

bool operator==(const CCycOcTreeNode& rhs) const
{
    return (rhs.value == value && rhs.color == color && rhs.object_class == object_class);
}

void copyData(const CCycOcTreeNode& from)
{
    OcTreeNode::copyData(from);
    this->color =  from.getColor();
    this->object_class = from.getObjectClass();
}

inline Color getColor() const { return color; }
inline void  setColor(Color c) {this->color = c; }
inline void  setColor(uint8_t r, uint8_t g, uint8_t b)
{
    this->color = Color(r,g,b); 
}

Color& getColor() { return color; }

// has any color been integrated? (pure white is very unlikely...)
inline bool isColorSet() const 
{ 
    return ((color.r != 255) || (color.g != 255) || (color.b != 255)); 
}

// Object class
inline int getObjectClass() const { return object_class; };
int& getObjectClass() { return object_class; };
inline void setObjectClass(int _cls) { this->object_class = _cls; };

void updateColorChildren();

CCycOcTreeNode::Color getAverageChildColor() const;

// file I/O
std::istream& readData(std::istream &s);
std::ostream& writeData(std::ostream &s) const;

protected:
Color color;

// CyC miscs
int object_class = -1;
};


// tree definition
class CCycOcTree : public octomap::OccupancyOcTreeBase <CCycOcTreeNode> 
{
public:
/// Default constructor, sets resolution of leafs
CCycOcTree(double resolution);

/// virtual constructor: creates a new object of same type
/// (Covariant return type requires an up-to-date compiler)
CCycOcTree* create() const { return new CCycOcTree(resolution); }

std::string getTreeType() const {return "CCycOcTree";}

/**
* Prunes a node when it is collapsible. This overloaded
* version only considers the node occupancy for pruning,
* different colors of child nodes are ignored.
* @return true if pruning was successful
*/
virtual bool pruneNode(CCycOcTreeNode* node);

virtual bool isNodeCollapsible(const CCycOcTreeNode* node) const;

// set node color at given key or coordinate. Replaces previous color.
CCycOcTreeNode* setNodeColor(const octomap::OcTreeKey& key, uint8_t r,  uint8_t g, uint8_t b);

CCycOcTreeNode* setNodeColor(float x, float y, float z, uint8_t r, uint8_t g, uint8_t b) 
{
    octomap::OcTreeKey key;
    if (!this->coordToKeyChecked(octomap::point3d(x,y,z), key)) return NULL;
        return setNodeColor(key,r,g,b);
}

// integrate color measurement at given key or coordinate. Average with previous color
CCycOcTreeNode* averageNodeColor(const octomap::OcTreeKey& key, uint8_t r, uint8_t g, uint8_t b);

CCycOcTreeNode* averageNodeColor(float x, float y, float z, uint8_t r, uint8_t g, uint8_t b) 
{
    octomap::OcTreeKey key;
    if (!this->coordToKeyChecked(octomap::point3d(x,y,z), key)) return NULL;
        return averageNodeColor(key,r,g,b);
}

// integrate color measurement at given key or coordinate. Average with previous color
CCycOcTreeNode* integrateNodeColor(const octomap::OcTreeKey& key, uint8_t r, uint8_t g, uint8_t b);

CCycOcTreeNode* integrateNodeColor(float x, float y, float z, uint8_t r, uint8_t g, uint8_t b) 
{
    octomap::OcTreeKey key;
    if (!this->coordToKeyChecked(octomap::point3d(x,y,z), key)) return NULL;
        return integrateNodeColor(key,r,g,b);
}

// update inner nodes, sets color to average child color
void updateInnerOccupancy();

// uses gnuplot to plot a RGB histogram in EPS format
void writeColorHistogram(std::string filename);

protected:
void updateInnerOccupancyRecurs(CCycOcTreeNode* node, unsigned int depth);

/**
* Static member object which ensures that this OcTree's prototype
* ends up in the classIDMapping only once. You need this as a 
* static member in any derived octree class in order to read .ot
* files through the AbstractOcTree factory. You should also call
* ensureLinking() once from the constructor.
*/
class StaticMemberInitializer
{
public:
    StaticMemberInitializer() 
    {
        CCycOcTree* tree = new CCycOcTree(0.1);
        tree->clearKeyRays();
        AbstractOcTree::registerTreeType(tree);
    }

    /**
    * Dummy function to ensure that MSVC does not drop the
    * StaticMemberInitializer, causing this tree failing to register.
    * Needs to be called from the constructor of this octree.
    */
    void ensureLinking() {};
};
/// static member to ensure static initialization (only once)
static StaticMemberInitializer CCycOcTreeMemberInit;
};

//! user friendly output in format (r g b)
std::ostream& operator<<(std::ostream& out, CCycOcTreeNode::Color const& c);

#endif

