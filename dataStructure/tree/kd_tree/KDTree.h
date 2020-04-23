#include "Point.h"

template<typename coordinate_type, size_t dimensions>
class kdtree{
public:
    typedef point<coordinate_type, dimensions> point_type;
private:
    struct node{
        node(const point_type& pt) : point_(pt), left_(nullptr), right_(nullptr){}

        coordinate_type get(size_t index) const{
            return point_.get(index);
        }

        double distance(const point_type& pt) const{
            return point_.distance(pt);
        }
        point_type point_;
        node* left_;
        node* right_;
    };
    node* root_;
    node* best_;
    double best_dist_;
    size_t visited_;
    std::vector<node> nodes_;
 
    node* make_tree(size_t begin, size_t end, size_t index);
 
    void knnSearch(node* root, const point_type& point, size_t index);
public:

    template<typename iterator>
    kdtree(iterator begin, iterator end);

    bool empty() const{return nodes_.empty();}
 
    size_t visited() const{return visited_;}
 
    double distance() const{return std::sqrt(best_dist_);}
 
    const point_type& knnSearch(const point_type& pt);

};

template<typename coordinate_type, size_t dimensions>
typename kdtree<coordinate_type,dimensions>::node* kdtree<coordinate_type,dimensions>::make_tree(size_t begin, size_t end, size_t index){
    if (end <= begin)
        return nullptr;
    size_t n = begin + (end - begin)/2;

    auto node_cmp = [index](const node& n1, const node& n2){
        return n1.point_.get(index) < n2.point_.get(index);
    };

    std::nth_element(&nodes_[begin], &nodes_[n], &nodes_[end], node_cmp);
    index = (index + 1) % dimensions;
    nodes_[n].left_ = make_tree(begin, n, index);
    nodes_[n].right_ = make_tree(n + 1, end, index);
    return &nodes_[n];
}

// constructor to initialize the tree
template<typename coordinate_type, size_t dimensions>
template<typename iterator>
kdtree<coordinate_type,dimensions>::kdtree(iterator begin, iterator end){
    best_ = nullptr;
    best_dist_ = 0;
    visited_ = 0;
    nodes_.reserve(std::distance(begin, end));
    for (auto i = begin; i != end; ++i)
        nodes_.emplace_back(*i);
    root_ = make_tree(0, nodes_.size(), 0);
}


template<typename coordinate_type, size_t dimensions>
void kdtree<coordinate_type,dimensions>::knnSearch(node* root, const point_type& point, size_t index){
    if (root == nullptr)
        return;
    ++visited_;
    double d = root->distance(point);
    if (best_ == nullptr || d < best_dist_){
        best_dist_ = d;
        best_ = root;
    }
    if (best_dist_ == 0)
        return;
    double dx = root->get(index) - point.get(index);
    index = (index + 1) % dimensions;
    knnSearch(dx > 0 ? root->left_ : root->right_, point, index);
    if (dx * dx >= best_dist_) //if doesn't find
        return;
    knnSearch(dx > 0 ? root->right_ : root->left_, point, index);
}

template<typename coordinate_type, size_t dimensions>
const typename kdtree<coordinate_type,dimensions>::point_type& kdtree<coordinate_type,dimensions>::knnSearch(const point_type& pt)
{
    if (root_ == nullptr)
        throw std::logic_error("tree is empty");
    best_ = nullptr;
    visited_ = 0;
    best_dist_ = 0;
    knnSearch(root_, pt, 0);
    return best_->point_;
}