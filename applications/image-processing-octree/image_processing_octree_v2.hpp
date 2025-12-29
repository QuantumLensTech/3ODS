// image_processing_octree_v2.hpp - OCTREE 3D CORRIGÉ
#pragma once

#include <cstdint>
#include <vector>
#include <array>
#include <memory>
#include <cmath>
#include <algorithm>
#include <functional>

namespace ods {
namespace image_processing {

/**
 * @brief Bounding Box 2D pour requêtes spatiales
 */
struct BBox {
    uint16_t min_x, min_y;
    uint16_t max_x, max_y;
    
    BBox() : min_x(0), min_y(0), max_x(0), max_y(0) {}
    BBox(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
        : min_x(x1), min_y(y1), max_x(x2), max_y(y2) {}
    
    bool contains(uint16_t x, uint16_t y) const {
        return x >= min_x && x <= max_x && y >= min_y && y <= max_y;
    }
    
    bool intersects(const BBox& other) const {
        return !(max_x < other.min_x || min_x > other.max_x ||
                 max_y < other.min_y || min_y > other.max_y);
    }
    
    uint16_t width() const { return max_x - min_x + 1; }
    uint16_t height() const { return max_y - min_y + 1; }
    uint32_t area() const { return static_cast<uint32_t>(width()) * height(); }
};

/**
 * @brief Pixel octovalent (x, y, intensité)
 */
struct OctoPixel {
    uint16_t x, y;
    uint8_t intensity;
    
    OctoPixel() : x(0), y(0), intensity(0) {}
    OctoPixel(uint16_t x_, uint16_t y_, uint8_t i) : x(x_), y(y_), intensity(i) {}
    
    bool operator==(const OctoPixel& other) const {
        return x == other.x && y == other.y && intensity == other.intensity;
    }
};

/**
 * @brief Nœud Octree 2D (quadtree en réalité, mais concept octree pour cohérence 3ODS)
 * 
 * Subdivision en 4 quadrants pour images 2D :
 *   [0] : NW (Nord-Ouest)
 *   [1] : NE (Nord-Est)  
 *   [2] : SW (Sud-Ouest)
 *   [3] : SE (Sud-Est)
 */
class OctreeNode {
public:
    static constexpr size_t MAX_PIXELS_PER_NODE = 8;  // Threshold pour subdivision
    static constexpr size_t NUM_CHILDREN = 4;          // Quadtree (2D)
    
    OctreeNode(const BBox& bounds) 
        : bounds_(bounds), is_leaf_(true) {
        pixels_.reserve(MAX_PIXELS_PER_NODE);
    }
    
    // Insertion pixel
    bool insert(const OctoPixel& pixel) {
        // Vérifier si pixel dans bounds
        if (!bounds_.contains(pixel.x, pixel.y)) {
            return false;
        }
        
        // Si pas feuille, déléguer aux enfants
        if (!is_leaf_) {
            for (auto& child : children_) {
                if (child && child->insert(pixel)) {
                    return true;
                }
            }
            return false;  // Aucun enfant n'accepte (ne devrait pas arriver)
        }
        
        // Vérifier si pixel existe déjà (update)
        for (auto& p : pixels_) {
            if (p.x == pixel.x && p.y == pixel.y) {
                p.intensity = pixel.intensity;  // Update
                return true;
            }
        }
        
        // Ajouter nouveau pixel
        pixels_.push_back(pixel);
        
        // Subdiviser si threshold dépassé
        if (pixels_.size() > MAX_PIXELS_PER_NODE && can_subdivide()) {
            subdivide();
        }
        
        return true;
    }
    
    // Recherche pixel exact
    bool find(uint16_t x, uint16_t y, uint8_t& out_intensity) const {
        if (!bounds_.contains(x, y)) {
            return false;
        }
        
        if (is_leaf_) {
            for (const auto& p : pixels_) {
                if (p.x == x && p.y == y) {
                    out_intensity = p.intensity;
                    return true;
                }
            }
            return false;
        }
        
        // Chercher dans enfants
        for (const auto& child : children_) {
            if (child && child->find(x, y, out_intensity)) {
                return true;
            }
        }
        
        return false;
    }
    
    // Requête rectangle
    void query_rect(const BBox& query, std::vector<OctoPixel>& results) const {
        // Pas d'intersection : stop
        if (!bounds_.intersects(query)) {
            return;
        }
        
        if (is_leaf_) {
            // Ajouter pixels dans query
            for (const auto& p : pixels_) {
                if (query.contains(p.x, p.y)) {
                    results.push_back(p);
                }
            }
        } else {
            // Récursion enfants
            for (const auto& child : children_) {
                if (child) {
                    child->query_rect(query, results);
                }
            }
        }
    }
    
    // Statistiques
    size_t count_pixels() const {
        if (is_leaf_) {
            return pixels_.size();
        }
        
        size_t total = 0;
        for (const auto& child : children_) {
            if (child) {
                total += child->count_pixels();
            }
        }
        return total;
    }
    
    size_t count_nodes() const {
        if (is_leaf_) {
            return 1;
        }
        
        size_t total = 1;  // Ce nœud
        for (const auto& child : children_) {
            if (child) {
                total += child->count_nodes();
            }
        }
        return total;
    }
    
    size_t count_leaves() const {
        if (is_leaf_) {
            return 1;
        }
        
        size_t total = 0;
        for (const auto& child : children_) {
            if (child) {
                total += child->count_leaves();
            }
        }
        return total;
    }
    
    uint8_t max_depth() const {
        if (is_leaf_) {
            return 0;
        }
        
        uint8_t max_child_depth = 0;
        for (const auto& child : children_) {
            if (child) {
                max_child_depth = std::max(max_child_depth, child->max_depth());
            }
        }
        return max_child_depth + 1;
    }
    
    const BBox& bounds() const { return bounds_; }
    bool is_leaf() const { return is_leaf_; }
    
private:
    BBox bounds_;
    bool is_leaf_;
    std::vector<OctoPixel> pixels_;  // Si feuille
    std::array<std::unique_ptr<OctreeNode>, NUM_CHILDREN> children_;  // Si non-feuille
    
    bool can_subdivide() const {
        // Vérifier taille minimale (au moins 2×2)
        return bounds_.width() >= 2 && bounds_.height() >= 2;
    }
    
    void subdivide() {
        if (!can_subdivide() || !is_leaf_) {
            return;
        }
        
        // Calculer centre
        uint16_t mid_x = bounds_.min_x + bounds_.width() / 2;
        uint16_t mid_y = bounds_.min_y + bounds_.height() / 2;
        
        // Créer 4 quadrants
        // [0] NW : (min_x, min_y) → (mid_x, mid_y)
        children_[0] = std::make_unique<OctreeNode>(
            BBox(bounds_.min_x, bounds_.min_y, mid_x, mid_y)
        );
        
        // [1] NE : (mid_x+1, min_y) → (max_x, mid_y)
        children_[1] = std::make_unique<OctreeNode>(
            BBox(mid_x + 1, bounds_.min_y, bounds_.max_x, mid_y)
        );
        
        // [2] SW : (min_x, mid_y+1) → (mid_x, max_y)
        children_[2] = std::make_unique<OctreeNode>(
            BBox(bounds_.min_x, mid_y + 1, mid_x, bounds_.max_y)
        );
        
        // [3] SE : (mid_x+1, mid_y+1) → (max_x, max_y)
        children_[3] = std::make_unique<OctreeNode>(
            BBox(mid_x + 1, mid_y + 1, bounds_.max_x, bounds_.max_y)
        );
        
        // Redistribuer pixels existants
        for (const auto& pixel : pixels_) {
            for (auto& child : children_) {
                if (child && child->insert(pixel)) {
                    break;
                }
            }
        }
        
        // Libérer mémoire pixels (plus une feuille)
        pixels_.clear();
        pixels_.shrink_to_fit();
        is_leaf_ = false;
    }
};

/**
 * @brief Image octovalente avec octree spatial
 */
class OctoImage {
public:
    OctoImage(uint16_t width, uint16_t height)
        : width_(width), height_(height),
          root_(std::make_unique<OctreeNode>(BBox(0, 0, width - 1, height - 1))) {}
    
    // Dimensions
    uint16_t width() const { return width_; }
    uint16_t height() const { return height_; }
    
    // Pixels
    void set_pixel(uint16_t x, uint16_t y, uint8_t intensity) {
        if (x >= width_ || y >= height_) {
            return;  // Out of bounds
        }
        
        if (intensity == 0) {
            // Ne pas stocker pixels noirs (sparse)
            return;
        }
        
        root_->insert(OctoPixel(x, y, intensity));
    }
    
    uint8_t get_pixel(uint16_t x, uint16_t y) const {
        if (x >= width_ || y >= height_) {
            return 0;
        }
        
        uint8_t intensity = 0;
        root_->find(x, y, intensity);
        return intensity;
    }
    
    // Requêtes spatiales
    std::vector<OctoPixel> query_rect(uint16_t x1, uint16_t y1, 
                                      uint16_t x2, uint16_t y2) const {
        std::vector<OctoPixel> results;
        BBox query(x1, y1, x2, y2);
        root_->query_rect(query, results);
        return results;
    }
    
    std::vector<OctoPixel> get_all_pixels() const {
        return query_rect(0, 0, width_ - 1, height_ - 1);
    }
    
    // Statistiques
    size_t num_pixels() const {
        return root_->count_pixels();
    }
    
    size_t num_nodes() const {
        return root_->count_nodes();
    }
    
    size_t num_leaves() const {
        return root_->count_leaves();
    }
    
    uint8_t tree_depth() const {
        return root_->max_depth();
    }
    
    float sparsity() const {
        size_t total = static_cast<size_t>(width_) * height_;
        return 1.0f - static_cast<float>(num_pixels()) / total;
    }
    
    float compression_ratio() const {
        // Mémoire non-compressée
        size_t uncompressed = static_cast<size_t>(width_) * height_;
        
        // Mémoire octree (estimation)
        size_t compressed = num_nodes() * sizeof(OctreeNode) + 
                           num_pixels() * sizeof(OctoPixel);
        
        return static_cast<float>(uncompressed) / (compressed / 1024.0f);
    }
    
    // Opérations images
    void clear() {
        root_ = std::make_unique<OctreeNode>(BBox(0, 0, width_ - 1, height_ - 1));
    }
    
    void invert() {
        auto pixels = get_all_pixels();
        clear();
        
        for (const auto& p : pixels) {
            set_pixel(p.x, p.y, 255 - p.intensity);
        }
    }
    
    void threshold(uint8_t threshold_value) {
        auto pixels = get_all_pixels();
        clear();
        
        for (const auto& p : pixels) {
            uint8_t new_intensity = (p.intensity >= threshold_value) ? 255 : 0;
            if (new_intensity > 0) {
                set_pixel(p.x, p.y, new_intensity);
            }
        }
    }
    
    // Factory : depuis buffer grayscale
    static OctoImage from_grayscale(const uint8_t* data, 
                                     uint16_t width, uint16_t height) {
        OctoImage img(width, height);
        
        for (uint16_t y = 0; y < height; ++y) {
            for (uint16_t x = 0; x < width; ++x) {
                uint8_t intensity = data[y * width + x];
                if (intensity > 0) {  // Sparse : skip black pixels
                    img.set_pixel(x, y, intensity);
                }
            }
        }
        
        return img;
    }
    
    // Export vers buffer
    void to_grayscale(uint8_t* data) const {
        // Initialiser à noir
        std::fill_n(data, static_cast<size_t>(width_) * height_, 0);
        
        // Remplir pixels non-zéro
        auto pixels = get_all_pixels();
        for (const auto& p : pixels) {
            data[p.y * width_ + p.x] = p.intensity;
        }
    }
    
private:
    uint16_t width_, height_;
    std::unique_ptr<OctreeNode> root_;
};

}} // namespace ods::image_processing
