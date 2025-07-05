#include "raylib.h"
#include "raymath.h"
#include <stdlib.h> 
#include <time.h>   
#include <math.h>  

// defines
#define MAX_SPHERES 100
#define MAX_VELOCITY 25.0f // max initial velocity
#define MAX_SPACE 50.0f // dimensions for the box

#define MIN_RADIUS 0.3f
#define MAX_RADIUS 10.0f
#define DENSITY 1.0f

#define WIDTH 1920
#define HEIGHT 1080

// struct for sphere 
typedef struct {
    Vector3 position;
    Vector3 velocity;
    Color color;
    float radius;
    float mass;
} Sphere;

// global variables 
Sphere spheres[MAX_SPHERES];
int numSpheres = 10; // num of spheres
float restCoeff = 0.9;
Sound hitSphere;
Sound hitWall;

// activates spheres
void ActivateSphere(Sphere* s) {
    s->radius = MIN_RADIUS + ((float)rand() / RAND_MAX) * (MAX_RADIUS - MIN_RADIUS);
    s->mass = DENSITY * (4.0f / 3.0f) * PI * powf(s->radius, 3);
    
    // spawns within bounds
    s->position.x = ((float)rand() / RAND_MAX) * (MAX_SPACE - s->radius * 4) - MAX_SPACE / 2.0f + s->radius * 2;    // all that (radius * 4...) math makes a buffer zone around the edges
    s->position.y = ((float)rand() / RAND_MAX) * (MAX_SPACE - s->radius * 4) - MAX_SPACE / 2.0f + s->radius * 2;
    s->position.z = ((float)rand() / RAND_MAX) * (MAX_SPACE - s->radius * 4) - MAX_SPACE / 2.0f + s->radius * 2;

    s->velocity.x = ((float)rand() / RAND_MAX) * MAX_VELOCITY * 2 - MAX_VELOCITY;
    s->velocity.y = ((float)rand() / RAND_MAX) * MAX_VELOCITY * 2 - MAX_VELOCITY;
    s->velocity.z = ((float)rand() / RAND_MAX) * MAX_VELOCITY * 2 - MAX_VELOCITY;

    s->color = (Color){ rand() % 256, rand() % 256, rand() % 256, 255 };
}

// resolves colision
void ResolveCollision(Sphere* sphereA, Sphere* sphereB) {
    // normal vector from the center of sphere A to B
    Vector3 normal = Vector3Subtract(sphereB->position, sphereA->position);
    float distSq = Vector3LengthSqr(normal); // distance squared
    float sumRadii = sphereA->radius + sphereB->radius;
    float minDistSq = sumRadii * sumRadii; // sum of radii squared

    //  separate spheres if they are overlapping
    if (distSq < minDistSq) {
        float dist = sqrtf(distSq);

        if (dist == 0) { // rare case that they are in the same spot
            // create a random normal to separate them
            normal = (Vector3){ (float)rand()/RAND_MAX - 0.5f, (float)rand()/RAND_MAX - 0.5f, (float)rand()/RAND_MAX - 0.5f };
            normal = Vector3Normalize(normal);
            dist = sumRadii; // assume they are at minimum contact point
        } else {
            normal = Vector3Scale(normal, 1.0f / dist); // normalize the vecto (show optimized vector normalization algorithm)
        }

        float overlap = sumRadii - dist; // how much spheres are overlapping
        // move them apart half of the overlap each
        Vector3 separation = Vector3Scale(normal, overlap / 2.0f);

        sphereA->position = Vector3Subtract(sphereA->position, separation);
        sphereB->position = Vector3Add(sphereB->position, separation);
    } else {
        // if no overlap, just normalize the normal vector (important if detection was for spheres *almost* touching)
        normal = Vector3Normalize(normal);
    }

    // calculate relative velocities along the normal before collision
    float velA_normal = Vector3DotProduct(sphereA->velocity, normal);
    float velB_normal = Vector3DotProduct(sphereB->velocity, normal);

    // if spheres are already moving away from each other, no need to calculate collision (I have the impression that if I this if this fucks things up)
    if (velA_normal - velB_normal > 0) 
        return; 

    // calculate new normal velocities using 1D collision formulas  (normal formulas, unfortunately)
    float totalMass = sphereA->mass + sphereB->mass;

    float vA_normal_final = (sphereA->mass * velA_normal + sphereB->mass * velB_normal - sphereB->mass * restCoeff * (velA_normal - velB_normal)) / totalMass;
    float vB_normal_final = (sphereA->mass * velA_normal + sphereB->mass * velB_normal - sphereA->mass * restCoeff * (velB_normal - velA_normal)) / totalMass;

    // calculate tangential velocities (components perpendicular to the normal)
    // do not change during collision
    Vector3 velA_tangential = Vector3Subtract(sphereA->velocity, Vector3Scale(normal, velA_normal));
    Vector3 velB_tangential = Vector3Subtract(sphereB->velocity, Vector3Scale(normal, velB_normal));

    // recompose final velocities of spheres
    sphereA->velocity = Vector3Add(Vector3Scale(normal, vA_normal_final), velA_tangential);
    sphereB->velocity = Vector3Add(Vector3Scale(normal, vB_normal_final), velB_tangential);
    
    PlaySound(hitSphere);
}

// detects and resolves colisions
void DetectAndResolveCollision() {
    for (int i = 0; i < numSpheres; i++) {
        for (int j = i + 1; j < numSpheres; j++) { // iterate over unique pairs of spheres
            float distance = Vector3Distance(spheres[i].position, spheres[j].position);
            float sumRadii = spheres[i].radius + spheres[j].radius;

            if (distance <= sumRadii)
                ResolveCollision(&spheres[i], &spheres[j]);     // collision detected, resolve it
        }
    }
}

// called at startup
void SetupInitialSpheres() {
    srand((unsigned int)time(NULL)); 
    for (int i = 0; i < numSpheres; i++) { // initialize only the starting number of spheres
        ActivateSphere(&spheres[i]);
    }
}

// draws spheres
void DrawBall() {    // not named DrawSphere cause theres already a drawsphere function
    for (int i = 0; i < numSpheres; i++)
        DrawSphere(spheres[i].position, spheres[i].radius, spheres[i].color);    
}

// destroys sphere
void DestroySphere(int index) {
    if (index >= 0 && index < MAX_SPHERES) {
        spheres[index].position = (Vector3){0.0f, 0.0f, 0.0f}; // resets position
        spheres[index].velocity = (Vector3){0.0f, 0.0f, 0.0f}; // stop its movement
        spheres[index].radius = 0.0f; // makes it invisible
        spheres[index].mass = 0.0f; // reset mass
        spheres[index].color = BLANK; // sets color to blank (kinda redundant but i dont think it has a great impact on performance)
    }
}

// calculates total energy of the system (just kinetic no gravitational)
float CalculateTotalEnergy() {
    float totalEnergy = 0.0f;
    for (int i = 0; i < numSpheres; i++) {
        float speed = Vector3Length(spheres[i].velocity);
        totalEnergy += 0.5f * spheres[i].mass * (speed * speed);
    }
    return totalEnergy;
}

// updates spheres positions and resolves colisions inside the box
void UpdateSim(float dt) {
    float min_bound = -MAX_SPACE / 2.0f;
    float max_bound = MAX_SPACE / 2.0f;

    for (int i = 0; i < numSpheres; i++) {
        // updates position
        spheres[i].position = Vector3Add(spheres[i].position, Vector3Scale(spheres[i].velocity, dt));

        // sphere collisions
        DetectAndResolveCollision();

        // colision with box
        // X axis
        if (spheres[i].position.x + spheres[i].radius > max_bound) {
            spheres[i].velocity.x *= -1;
            spheres[i].position.x = max_bound - spheres[i].radius; // adjusts if it goes out of bounds
            PlaySound(hitWall);
        } 
        else if (spheres[i].position.x - spheres[i].radius < min_bound) {
            spheres[i].velocity.x *= -1;
            spheres[i].position.x = min_bound + spheres[i].radius;
            PlaySound(hitWall);
        }

        // Y axis
        if (spheres[i].position.y + spheres[i].radius > max_bound) {
            spheres[i].velocity.y *= -1;
            spheres[i].position.y = max_bound - spheres[i].radius;
            PlaySound(hitWall);
        } 
        else if (spheres[i].position.y - spheres[i].radius < min_bound) {
            spheres[i].velocity.y *= -1;
            spheres[i].position.y = min_bound + spheres[i].radius;
            PlaySound(hitWall);
        }

        // Z axis
        if (spheres[i].position.z + spheres[i].radius > max_bound) {
            spheres[i].velocity.z *= -1;
            spheres[i].position.z = max_bound - spheres[i].radius;
            PlaySound(hitWall);
        } 
        else if (spheres[i].position.z - spheres[i].radius < min_bound) {
            spheres[i].velocity.z *= -1;
            spheres[i].position.z = min_bound + spheres[i].radius;
            PlaySound(hitWall);
        }
    }
}



int main(){
    // initialization
    const int screenWidth = WIDTH;
    const int screenHeight = HEIGHT; // size of window

    InitWindow(screenWidth, screenHeight, "3d sim");
    InitAudioDevice();

    // 3D camera (taken from raylib examples)
    Camera3D camera = { 0 };
    camera.position = (Vector3){ 40.0f, 40.0f, 40.0f }; // camera position
    camera.target = (Vector3){ 0.0f, 0.0f, 0.0f };      // camera looking at point 
    camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };          // camera up vector 
    camera.fovy = 45.0f;                                // camera fov Y
    camera.projection = CAMERA_PERSPECTIVE;             // camera projection type

    hitSphere = LoadSound("hitball.wav");
    hitWall = LoadSound("hitwall.wav");

    DisableCursor();    // cursor inside window

    //SetTargetFPS(60);                 // sets game to run at target fps (explain in video)
    

    // initializes spheres
    SetupInitialSpheres();

    // main game loop
    while (!WindowShouldClose())        // detect window close button or esc key
    {
        // update
        //----------------------------------------------------------------------------------
        UpdateCamera(&camera, CAMERA_FREE);

        // resets camera
        if (IsKeyPressed(KEY_R)) { // use R to reset camera
            camera.position = (Vector3){ 10.0f, 10.0f, 10.0f };
            camera.target = (Vector3){ 0.0f, 0.0f, 0.0f };
            camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };
        }
        // updates rest coeff
        if (IsKeyPressed(KEY_EQUAL))
            restCoeff += 0.05f;
        if (IsKeyPressed(KEY_MINUS) && restCoeff > 0)    
            restCoeff -= 0.05f;
        
        // updates n of spheres
        if (IsKeyPressed(KEY_M) && numSpheres < MAX_SPHERES){
            ActivateSphere(&spheres[numSpheres]);
            numSpheres++;
        }
        if (IsKeyPressed(KEY_N) && numSpheres > 1)
            numSpheres--;
            DestroySphere(numSpheres);
        // total energy
        float energy = CalculateTotalEnergy();

        // updates spheres sim
        float deltaTime = GetFrameTime();
        UpdateSim(deltaTime);
        //----------------------------------------------------------------------------------

        // draw
        //----------------------------------------------------------------------------------
        BeginDrawing();

            ClearBackground(BLACK);

            BeginMode3D(camera);

                // draws spheres
                DrawBall();

                // draws box
                DrawCubeWires((Vector3){ 0.0f, 0.0f, 0.0f }, MAX_SPACE, MAX_SPACE, MAX_SPACE, WHITE);
                // draw grid on the floor (just for looks)
                //DrawGrid(0, 0.0f);


            EndMode3D();

            // fps and misc info
            DrawFPS(10, 10);
            DrawText("Free camera controls:", 10, 40, 20, WHITE);
            DrawText("- Move with WASD, Q/E for Up/Down", 10, 60, 20, WHITE);
            DrawText("- Rotate with Mouse (Hold Right Click)", 10, 80, 20, WHITE);
            DrawText("- Scroll Wheel for Zoom", 10, 100, 20, WHITE);
            DrawText("- Press 'R' to reset camera", 10, 120, 20, WHITE);
            DrawText(TextFormat("Restitution Coeff: %.2f (Press +/-)", restCoeff), 10, 160, 20, WHITE);
            DrawText(TextFormat("Spheres: %i (Press N/M)", numSpheres), 10, 140, 20, WHITE);
            DrawText(TextFormat("Total Energy: %.2f J", energy), 10, 180, 20, WHITE);

        EndDrawing();
        //----------------------------------------------------------------------------------
    }

    // de initialization
    //--------------------------------------------------------------------------------------
    UnloadSound(hitSphere);
    UnloadSound(hitWall);
    CloseAudioDevice();
    CloseWindow();        // close window
    //--------------------------------------------------------------------------------------

     return 0;
}



/* some cool code
float Q_rsqrt( float number )
{
	long i;
	float x2, y;
	const float threehalfs = 1.5F;

	x2 = number * 0.5F;
	y  = number;
	i  = * ( long * ) &y;                       // evil floating point bit level hacking
	i  = 0x5f3759df - ( i >> 1 );               // what the fuck?
	y  = * ( float * ) &i;
	y  = y * ( threehalfs - ( x2 * y * y ) );   // 1st iteration
//	y  = y * ( threehalfs - ( x2 * y * y ) );   // 2nd iteration, this can be removed

	return y;
}
*/