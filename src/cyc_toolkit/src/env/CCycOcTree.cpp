/*
 * 
 */

#include <env/CCycOcTree.h>

// node implementation  --------------------------------------
std::ostream& CCycOcTreeNode::writeData(std::ostream &s) const
{
s.write((const char*) &value, sizeof(value)); // occupancy
s.write((const char*) &color, sizeof(Color)); // color
//s.write((const char*) &node_cls, sizeof(node_cls)); // object class
return s;
}

std::istream& CCycOcTreeNode::readData(std::istream &s) 
{
s.read((char*) &value, sizeof(value)); // occupancy
s.read((char*) &color, sizeof(Color)); // color
//s.read((char*) &node_cls, sizeof(node_cls)); // object class
return s;
}

CCycOcTreeNode::Color CCycOcTreeNode::getAverageChildColor() const 
{
int mr = 0;
int mg = 0;
int mb = 0;
int c = 0;

if (children != NULL)
{
    for (int i=0; i<8; i++) 
    {
        CCycOcTreeNode* child = static_cast<CCycOcTreeNode*>(children[i]);

        if (child != NULL && child->isColorSet()) 
        {
            mr += child->getColor().r;
            mg += child->getColor().g;
            mb += child->getColor().b;
            ++c;
        }
    }
}

if (c > 0) 
{
    mr /= c;
    mg /= c;
    mb /= c;
    return Color((uint8_t) mr, (uint8_t) mg, (uint8_t) mb);
}
else 
{ 
    // no child had a color other than white
    return Color(255, 255, 255);
}
}


void CCycOcTreeNode::updateColorChildren()
{
color = getAverageChildColor();
}

// tree implementation  --------------------------------------
CCycOcTree::CCycOcTree(double in_resolution) : OccupancyOcTreeBase<CCycOcTreeNode>(in_resolution) 
{
CCycOcTreeMemberInit.ensureLinking();
}

CCycOcTreeNode* CCycOcTree::setNodeColor(const octomap::OcTreeKey& key, uint8_t r, uint8_t g, uint8_t b) 
{
CCycOcTreeNode* n = search (key);
if (n != 0) 
{
    n->setColor(r, g, b);
}
return n;
}

bool CCycOcTree::pruneNode(CCycOcTreeNode* node)
{
if (!isNodeCollapsible(node))
    return false;

// set value to children's values (all assumed equal)
node->copyData(*(getNodeChild(node, 0)));

if (node->isColorSet()) // TODO check
node->setColor(node->getAverageChildColor());

// delete children
for (unsigned int i=0;i<8;i++) 
{
    deleteNodeChild(node, i);
}
delete[] node->children;
node->children = NULL;

return true;
}

bool CCycOcTree::isNodeCollapsible(const CCycOcTreeNode* node) const
{
// all children must exist, must not have children of
// their own and have the same occupancy probability
if (!nodeChildExists(node, 0))
    return false;

const CCycOcTreeNode* firstChild = getNodeChild(node, 0);
if (nodeHasChildren(firstChild))
    return false;

for (unsigned int i = 1; i<8; i++) 
{
    // compare nodes only using their occupancy, ignoring color for pruning
    if (!nodeChildExists(node, i) || nodeHasChildren(getNodeChild(node, i)) || !(getNodeChild(node, i)->getValue() == firstChild->getValue()))
    return false;
}

return true;
}

CCycOcTreeNode* CCycOcTree::averageNodeColor(const octomap::OcTreeKey& key, uint8_t r, uint8_t g, uint8_t b) 
{
CCycOcTreeNode* n = search(key);
if (n != 0) 
{
    if (n->isColorSet()) 
    {
        CCycOcTreeNode::Color prev_color = n->getColor();
        n->setColor((prev_color.r + r)/2, (prev_color.g + g)/2, (prev_color.b + b)/2);
    }
    else 
    {
        n->setColor(r, g, b);
    }
}
return n;
}

CCycOcTreeNode* CCycOcTree::integrateNodeColor(const octomap::OcTreeKey& key, uint8_t r, uint8_t g, uint8_t b) 
{
CCycOcTreeNode* n = search (key);
if (n != 0) 
{
    if (n->isColorSet()) 
    {
        CCycOcTreeNode::Color prev_color = n->getColor();
        double node_prob = n->getOccupancy();
        uint8_t new_r = (uint8_t) ((double) prev_color.r * node_prob +  (double) r * (0.99-node_prob));
        uint8_t new_g = (uint8_t) ((double) prev_color.g * node_prob +  (double) g * (0.99-node_prob));
        uint8_t new_b = (uint8_t) ((double) prev_color.b * node_prob +  (double) b * (0.99-node_prob));
        n->setColor(new_r, new_g, new_b);
    }
    else 
    {
        n->setColor(r, g, b);
    }
}
return n;
}


void CCycOcTree::updateInnerOccupancy() 
{
this->updateInnerOccupancyRecurs(this->root, 0);
}

void CCycOcTree::updateInnerOccupancyRecurs(CCycOcTreeNode* node, unsigned int depth) 
{
// only recurse and update for inner nodes:
if (nodeHasChildren(node))
{
    // return early for last level:
    if (depth < this->tree_depth)
    {
        for (unsigned int i=0; i<8; i++) 
        {
            if (nodeChildExists(node, i)) 
            {
                updateInnerOccupancyRecurs(getNodeChild(node, i), depth+1);
            }
        }
    }
    node->updateOccupancyChildren();
    node->updateColorChildren();
}
}

void CCycOcTree::writeColorHistogram(std::string filename) 
{
#ifdef _MSC_VER
fprintf(stderr, "The color histogram uses gnuplot, this is not supported under windows.\n");
#else
// build RGB histogram
std::vector<int> histogram_r (256,0);
std::vector<int> histogram_g (256,0);
std::vector<int> histogram_b (256,0);
for(CCycOcTree::tree_iterator it = this->begin_tree(),
end=this->end_tree(); it!= end; ++it) {
if (!it.isLeaf() || !this->isNodeOccupied(*it)) continue;
CCycOcTreeNode::Color& c = it->getColor();
++histogram_r[c.r];
++histogram_g[c.g];
++histogram_b[c.b];
}
// plot data
FILE *gui = popen("gnuplot ", "w");
fprintf(gui, "set term postscript eps enhanced color\n");
fprintf(gui, "set output \"%s\"\n", filename.c_str());
fprintf(gui, "plot [-1:256] ");
fprintf(gui,"'-' w filledcurve lt 1 lc 1 tit \"r\",");
fprintf(gui, "'-' w filledcurve lt 1 lc 2 tit \"g\",");
fprintf(gui, "'-' w filledcurve lt 1 lc 3 tit \"b\",");
fprintf(gui, "'-' w l lt 1 lc 1 tit \"\",");
fprintf(gui, "'-' w l lt 1 lc 2 tit \"\",");
fprintf(gui, "'-' w l lt 1 lc 3 tit \"\"\n");

for (int i=0; i<256; ++i) fprintf(gui,"%d %d\n", i, histogram_r[i]);
fprintf(gui,"0 0\n"); fprintf(gui, "e\n");
for (int i=0; i<256; ++i) fprintf(gui,"%d %d\n", i, histogram_g[i]);
fprintf(gui,"0 0\n"); fprintf(gui, "e\n");
for (int i=0; i<256; ++i) fprintf(gui,"%d %d\n", i, histogram_b[i]);
fprintf(gui,"0 0\n"); fprintf(gui, "e\n");
for (int i=0; i<256; ++i) fprintf(gui,"%d %d\n", i, histogram_r[i]);
fprintf(gui, "e\n");
for (int i=0; i<256; ++i) fprintf(gui,"%d %d\n", i, histogram_g[i]);
fprintf(gui, "e\n");
for (int i=0; i<256; ++i) fprintf(gui,"%d %d\n", i, histogram_b[i]);
fprintf(gui, "e\n");
fflush(gui);
#endif
}

std::ostream& operator<<(std::ostream& out, CCycOcTreeNode::Color const& c) 
{
return out << '(' << (unsigned int)c.r << ' ' << (unsigned int)c.g << ' ' << (unsigned int)c.b << ')';
}

CCycOcTree::StaticMemberInitializer CCycOcTree::CCycOcTreeMemberInit;

