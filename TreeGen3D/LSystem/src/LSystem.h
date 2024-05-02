#ifndef LSystem_H_
#define LSystem_H_

#include <string>
#include <vector>
#include <map>
#include "vec.h"

class LSystem
{
public:
    typedef std::pair<vec3, std::string> Geometry;
    typedef std::pair<vec3, vec3> Branch;

public:
    LSystem();
    ~LSystem() {}

    // Set/get inputs
    void loadProgram(const std::string& fileName);
    void loadProgramFromString(const std::string& program);
    void setDefaultAngle(float degrees);
    void setDefaultStep(float distance);

    void setDefaultScalar(float scalar);
    void setDefaultAPB(float anglePerturbation);
    void setDefaultSSPB(float stepSizePerturbation);

    float getDefaultAngle() const;
    float getDefaultStep() const;
    float getDefaultScalar() const;
    float getDefaultAPB() const;
    float getDefaultSSPB() const;

    const std::string& getGrammarString() const;

    // Iterate grammar
    const std::string& getIteration(unsigned int n);

    // Get geometry from running the turtle
    void process(unsigned int n, 
        std::vector<Branch>& branches); 
    void process(unsigned int n, 
        std::vector<Branch>& branches, 
        std::vector<Geometry>& models);

protected:
    void reset();
    void addProduction(std::string line);
    std::string iterate(const std::string& input);
    
    std::map<std::string, std::string> productions;
    std::vector<std::string> iterations;
    std::vector<std::pair<vec3,vec3>> bboxes;
    std::string current;
    float mDfltAngle;
    float mDfltStep;
    float mDfltScalar;
    float mDfltAPB;
    float mDfltSSPB;
    std::string mGrammar;

    class Turtle
    {
    public:
        Turtle();
        Turtle(const Turtle& t);
        Turtle& operator=(const Turtle& t);

        void moveForward(float distance);
        void applyUpRot(float degrees);
        void applyLeftRot(float degrees);
        void applyForwardRot(float degrees);

        vec3 pos;
        vec3 up;
        vec3 forward;
        vec3 left;
    };
};

#endif