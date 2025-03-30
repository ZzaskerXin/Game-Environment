#include "Engine/gameobject.h"
#include "Engine/system.h"

class CollisionSystem : public System{
public:

    void setTerrain(const std::vector<float>& terrainData) {
        terrainVertices = terrainData;
    }

    std::vector<float> getTerrain() const {
        return terrainVertices;
    }

    glm::vec3 closestPointOnSegment(const glm::vec3& a, const glm::vec3& b, const glm::vec3& point) {
        glm::vec3 ab = b - a;
        float t = glm::dot(point - a, ab) / glm::dot(ab, ab);
        t = glm::clamp(t, 0.0f, 1.0f);
        return a + t * ab;
    }

    glm::vec3 closestPointOnTriangle(const glm::vec3& point, const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2) {
        glm::vec3 edge0 = v1 - v0;
        glm::vec3 edge1 = v2 - v0;
        glm::vec3 v0_to_point = point - v0;

        float dot00 = glm::dot(edge0, edge0);
        float dot01 = glm::dot(edge0, edge1);
        float dot02 = glm::dot(edge0, v0_to_point);
        float dot11 = glm::dot(edge1, edge1);
        float dot12 = glm::dot(edge1, v0_to_point);

        float denom = dot00 * dot11 - dot01 * dot01;

        if (denom == 0.0f) {
            return v0;
        }

        float u = (dot11 * dot02 - dot01 * dot12) / denom;
        float v = (dot00 * dot12 - dot01 * dot02) / denom;

        if (u >= 0 && v >= 0 && (u + v) <= 1) {
            return v0 + u * edge0 + v * edge1;
        }

        glm::vec3 closest = v0;
        float minDist = glm::distance(point, v0);

        glm::vec3 candidates[] = {
            closestPointOnSegment(v0, v1, point),
            closestPointOnSegment(v1, v2, point),
            closestPointOnSegment(v2, v0, point)
        };

        for (const auto& candidate : candidates) {
            float dist = glm::distance(point, candidate);
            if (dist < minDist) {
                minDist = dist;
                closest = candidate;
            }
        }

        return closest;
    }

    glm::vec3 ellipsoidCollisionResponse(const glm::vec3& objectPos, const glm::vec3& objectRadius, const std::vector<float>& vertices) {
        glm::vec3 closestPoint;
        float minDist = std::numeric_limits<float>::max();

        for (size_t i = 0; i < vertices.size(); i += 9) {
            glm::vec3 v0(vertices[i], vertices[i + 1], vertices[i + 2]);
            glm::vec3 v1(vertices[i + 3], vertices[i + 4], vertices[i + 5]);
            glm::vec3 v2(vertices[i + 6], vertices[i + 7], vertices[i + 8]);

            glm::vec3 point = closestPointOnTriangle(objectPos, v0, v1, v2);
            float dist = glm::distance(objectPos, point);

            if (dist < minDist) {
                minDist = dist;
                closestPoint = point;
            }
        }
        return closestPoint + objectRadius - objectPos;
    }


    glm::vec3 closestPointOnTerrain(const glm::vec3& position) {
        glm::vec3 closestPoint;
        float minDist = std::numeric_limits<float>::max();

        for (size_t i = 0; i < terrainVertices.size(); i += 9) {
            glm::vec3 v0(terrainVertices[i], terrainVertices[i + 1], terrainVertices[i + 2]);
            glm::vec3 v1(terrainVertices[i + 3], terrainVertices[i + 4], terrainVertices[i + 5]);
            glm::vec3 v2(terrainVertices[i + 6], terrainVertices[i + 7], terrainVertices[i + 8]);

            glm::vec3 point = closestPointOnTriangle(position, v0, v1, v2);
            float dist = glm::distance(position, point);

            if (dist < minDist) {
                minDist = dist;
                closestPoint = point;
            }
        }
        return closestPoint;
    }

    bool isCollidingWithGround(const glm::vec3& position, float radius) {
        glm::vec3 closestPoint = closestPointOnTerrain(position);
        return (position.y - radius) <= closestPoint.y;
    }
private:
    std::vector<float> terrainVertices;
};
