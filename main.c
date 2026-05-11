#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "raylib.h"
#include <math.h>

// ---- CONSTANTS ----

double LAMBDA1 = 2.5; // seperation (how much they stay seperated)
double LAMBDA2 = 1.0; // alignment (how much they align with neighbors)
double LAMBDA3 = 0.5; // cohesion (how much they group together)
double LAMBDA4 = 1.0; // random noise
double PERCEPTION_RADIUS = 30.0;
double SEPARATION_RADIUS = 10.0;
double VELOCITY_CONST = 2.0; // speed of the boids
double ACCELERATION_CONST = 0.35;
int BOID_MAX = 1500;
const int SCREEN_HEIGHT = 800;
const int SCREEN_WIDTH = 1200;
const int FPS = 30;

// ---- Structs ----
typedef struct {
  double x;
  double y;
  double vx;
  double vy;
} boid;

// ---- Prototypes ----
double l2_norm(Vector2 v);
Vector2 randomUnitVector(void);
Vector2 calculateCohesion(boid bk, int k, boid barr[], int boidCount);
Vector2 calculateSeperation(boid bk, int k, boid barr[], int boidCount);
Vector2 calculateBoidAlignment(boid bk, int k, boid barr[], int boidCount);
double to_radians(double deg);
void drawBoid(boid b, Color color);
void WriteCursorPosition(void);

// ---- IMPLEMENTATIONS ----

double l2_norm(Vector2 v){
  return sqrt(v.x * v.x + v.y * v.y);
}

Vector2 randomUnitVector(){
  double randx = (( rand() / (double) RAND_MAX) * 2) - 1;
  double randy = (( rand() / (double) RAND_MAX) * 2) - 1;  
  Vector2 v = {randx, randy};
  
  double norm = l2_norm(v);
  v.x /= norm;
  v.y /= norm;

  return v;  
}

Vector2 calculateCohesion(boid bk, int k, boid barr[], int boidCount){
  Vector2 center = {.x = 0, .y = 0};
  int count = 0;

  // loop through boids in world 
  for (int i = 0; i < boidCount; i++){
    if (i != k){
      boid bi = barr[i];

      double dx = bi.x - bk.x;
      double dy = bi.y - bk.y;
      double dist = sqrt(dx * dx + dy * dy);

      if (dist < PERCEPTION_RADIUS) {
        center.x += bi.x;
        center.y += bi.y;
        count++;
 
      }

    }
  }

  if (count == 0){
    return (Vector2){0,0};
  }

  center.x /= count;
  center.y /= count;

  Vector2 v = {
    .x = center.x - bk.x,
    .y = center.y - bk.y
  };

  double v_norm = l2_norm(v); 
  if (v_norm > 0) {
    v.x /= v_norm;
    v.y /= v_norm;
  }

  return v;

}

Vector2 calculateBoidSeperation(boid bk, int k, boid barr[], int boidCount){
  Vector2 v = {.x = 0, .y = 0};

  // loop through boids in world 
  for (int i = 0; i < boidCount; i++){
    if (i != k){
      boid bi = barr[i];
      double x = bk.x - bi.x;
      double y = bk.y - bi.y;
      
      double norm = pow(x,2) + pow(y, 2);
      if (norm > 0){
        x = x / norm;
        y = y / norm;
      }
  
      double dx = bi.x - bk.x;
      double dy = bi.y - bk.y;
      double dist = sqrt(dx * dx + dy * dy);
      if (dist < SEPARATION_RADIUS) {
        v.x += x;
        v.y += y;
      }
    }
  }

  double v_norm = l2_norm(v); 
  if (v_norm > 0) {
    v.x /= v_norm;
    v.y /= v_norm;
  }

  return v;
}

Vector2 calculateBoidAlignment(boid bk, int k, boid barr[], int boidCount){
  Vector2 v = {.x = 0, .y = 0};
  int count = 0;

  for (int i = 0; i < boidCount; i++){
    if (i != k){
      boid bi = barr[i]; 
      double dx = bi.x - bk.x;
      double dy = bi.y - bk.y;
      double dist = sqrt(dx * dx + dy * dy);

      if (dist < PERCEPTION_RADIUS) {
        v.x += bi.vx;
        v.y += bi.vy;
        count++;
      }
    }
  }

  if (count == 0){ return (Vector2) {0,0};}
  
  v.x /= count;
  v.y /= count;

  double v_norm = l2_norm(v);
  if (v_norm > 0){
    v.x /= v_norm;
    v.y /= v_norm;
  }
  return v;
}

double to_radians(double deg){
  return deg * M_PI / 180.0;
}
 
void drawBoid(boid b, Color color){
  double x = b.x;
  double y = b.y;
  double vx = b.vx;
  double vy = b.vy;

  int r = 8;

  double offsetDegree = atan2(vy,vx);

  Vector2 tip = {
    .x = x + r * cos(offsetDegree),
    .y = y + r * sin(offsetDegree),
  };

  Vector2 left = {
    .x = x + r * cos(offsetDegree + 2.5),
    .y = y + r * sin(offsetDegree + 2.5)
  };

  Vector2 right = {
    .x = x + r * cos(offsetDegree - 2.5),
    .y = y + r * sin(offsetDegree - 2.5)
  };

  DrawTriangle(tip, right, left, color); 
}   

void WriteCursorPosition(void){
    char mouse_position_text[50];
    int max_len = sizeof(mouse_position_text);
    snprintf(mouse_position_text, max_len, "Mouse Position\n x: %d\n y: %d", GetMouseX(), GetMouseY());    
    DrawText(mouse_position_text, 0, 0, 16, BLUE);
}

int main(void){

  const char *screenTitle = "Hello There!";
  InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, screenTitle);
  SetTargetFPS(FPS);


  boid boidarray[BOID_MAX];
  int boidCount = 0;


  while (!WindowShouldClose()){

    BeginDrawing();
      ClearBackground(RAYWHITE);
      WriteCursorPosition();
  
    if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON) || IsMouseButtonDown(MOUSE_LEFT_BUTTON)) {
      if (boidCount < BOID_MAX) {                                 
        Vector2 rv = randomUnitVector();
        boidarray[boidCount] = (boid) {
          .x = GetMouseX(), 
          .y = GetMouseY(),
          .vx = rv.x,
          .vy = rv.y
        };
        boidCount += 1;
      }
    } 

    // loop through existing boids
    for (int i = 0; i < boidCount; i++) {
      boid bk = boidarray[i];
      drawBoid(bk, BLUE);
      Vector2 s = calculateBoidSeperation(bk, i, boidarray, boidCount);
      Vector2 a = calculateBoidAlignment(bk, i, boidarray, boidCount);
      Vector2 c = calculateCohesion(bk, i, boidarray, boidCount);  
      Vector2 rv = randomUnitVector();

      Vector2 v = {
        .x = LAMBDA1 * s.x + LAMBDA2 * a.x + LAMBDA3 * c.x + LAMBDA4 * rv.x,
        .y = LAMBDA1 * s.y + LAMBDA2 * a.y + LAMBDA3 * c.y + LAMBDA4 * rv.y
      };


      boid *bka = &boidarray[i];
      bka->x += bka->vx;
      bka->y += bka->vy;
      bka->vx += ACCELERATION_CONST * v.x;
      bka->vy += ACCELERATION_CONST * v.y;

      // normalize velocities
      double norm = sqrt(pow(bka->vx, 2) + pow(bka->vy,2));
      if ( norm > 0){
        bka->vx = VELOCITY_CONST * (bka->vx / norm);
        bka->vy = VELOCITY_CONST * (bka->vy / norm);
      }
      
      DrawLine(bka->x, bka->y, bka->x + (5 * bka->vx), bka->y + (5*bka->vy), RED);

      if (bka->x > SCREEN_WIDTH){
        bka->x = 0.0;
      } 
      if (bka->x < 0.0){
        bka->x = SCREEN_WIDTH;
      }

      if (bka->y > SCREEN_HEIGHT){
        bka->y = 0.0;
      }
      if(bka->y < 0.0){
          bka->y = SCREEN_HEIGHT;
      }
    }


    EndDrawing();
  
  }

  CloseWindow();
  printf("Hello World");
}
