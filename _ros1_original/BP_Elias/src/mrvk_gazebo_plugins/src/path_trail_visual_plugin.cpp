// Gazebo Visual Plugin — kresli prejdenu trajektoriu robota priamo v
// Gazebo scene (gzclient). Cita svetovu pozu robota z rendering scene,
// takze nie je problem s frame mismatchom (odom vs world).
//
// Pripaja sa na <visual> v .world (typicky ground_plane). Body sa
// transformuju zo svetovych suradnic do lokalneho frame rodicovskeho
// visualu (kompenzacia jeho pozy + scale), takze funguje aj na
// ground_plane ktory ma scale ~50 z <plane><size>.
//
// SDF parametre:
//   <robot_name>   meno modelu v Gazebe   (default "robot")
//   <min_distance> min posun medzi bodmi  (default 0.05 m)
//   <line_z>       vyska ciary nad podlahou (default 0.05 m)
//   <line_offset>  kolme odsadenie paralelnej ciary (default 0.025 m).
//                  Vykresluju sa dve paralelne ciary s odsadenim +/- tejto
//                  hodnoty, cim vznikne efekt 2x hrubsej ciary (Ogre
//                  line strip nepodporuje line width).

#include <cmath>
#include <functional>
#include <string>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/rendering/DynamicLines.hh>

#include <ignition/math/Vector3.hh>

namespace gazebo
{
class PathTrailVisualPlugin : public VisualPlugin
{
public:
  void Load(rendering::VisualPtr _visual, sdf::ElementPtr _sdf) override
  {
    if (!_visual)
    {
      gzerr << "[PathTrail] Load() dostal NULL visual!\n";
      return;
    }
    this->visual_ = _visual;

    this->robot_name_ = _sdf->HasElement("robot_name")
                          ? _sdf->Get<std::string>("robot_name")
                          : std::string("robot");
    this->min_dist_ = _sdf->HasElement("min_distance")
                          ? _sdf->Get<double>("min_distance") : 0.05;
    this->line_z_ = _sdf->HasElement("line_z")
                          ? _sdf->Get<double>("line_z") : 0.05;
    this->line_offset_ = _sdf->HasElement("line_offset")
                          ? _sdf->Get<double>("line_offset") : 0.020;

    // Ciara sa vytvori na visuale na ktory je plugin pripojeny.
    // Ten MUSI mat scale 1 a byt v (0,0,0) (trail_anchor model),
    // inak by sa body skreslili (scale) alebo posunuli (offset).
    // Vytvarame tri paralelne ciary (krajne +/- line_offset_ + stredna)
    // s malym odsadenim, takze vznikne efekt suvislej hrubsej ciary.
    this->line_a_ = this->visual_->CreateDynamicLine(
        rendering::RENDERING_LINE_STRIP);
    this->line_a_->setMaterial("Gazebo/Purple");
    this->line_a_->setVisibilityFlags(GZ_VISIBILITY_GUI);

    this->line_b_ = this->visual_->CreateDynamicLine(
        rendering::RENDERING_LINE_STRIP);
    this->line_b_->setMaterial("Gazebo/Purple");
    this->line_b_->setVisibilityFlags(GZ_VISIBILITY_GUI);

    this->line_c_ = this->visual_->CreateDynamicLine(
        rendering::RENDERING_LINE_STRIP);
    this->line_c_->setMaterial("Gazebo/Purple");
    this->line_c_->setVisibilityFlags(GZ_VISIBILITY_GUI);

    this->line_d_ = this->visual_->CreateDynamicLine(
        rendering::RENDERING_LINE_STRIP);
    this->line_d_->setMaterial("Gazebo/Purple");
    this->line_d_->setVisibilityFlags(GZ_VISIBILITY_GUI);

    this->line_e_ = this->visual_->CreateDynamicLine(
        rendering::RENDERING_LINE_STRIP);
    this->line_e_->setMaterial("Gazebo/Purple");
    this->line_e_->setVisibilityFlags(GZ_VISIBILITY_GUI);

    this->update_conn_ = event::Events::ConnectPreRender(
        std::bind(&PathTrailVisualPlugin::OnPreRender, this));

    gzmsg << "[PathTrail] AKTIVNY. sledujem model '" << this->robot_name_
          << "' min_dist=" << this->min_dist_ << "\n";
  }

private:
  void OnPreRender()
  {
    rendering::ScenePtr scene = this->visual_->GetScene();
    if (!scene)
      return;

    rendering::VisualPtr robot = scene->GetVisual(this->robot_name_);
    if (!robot)
      return;

    const ignition::math::Vector3d p = robot->WorldPose().Pos();

    if (this->has_last_)
    {
      const double dx = p.X() - this->last_.X();
      const double dy = p.Y() - this->last_.Y();
      if (dx * dx + dy * dy < this->min_dist_ * this->min_dist_)
        return;
    }

    // Kolme odsadenie obidvoch ciar v rovine XY (perp k smeru pohybu).
    double px = 0.0, py = 0.0;
    if (this->has_last_) {
      const double dx = p.X() - this->last_.X();
      const double dy = p.Y() - this->last_.Y();
      const double len = std::sqrt(dx * dx + dy * dy);
      if (len > 1e-6) {
        px = -dy / len * this->line_offset_;
        py =  dx / len * this->line_offset_;
      }
    }

    const ignition::math::Vector3d world_a(
        p.X() + px, p.Y() + py, this->line_z_);
    const ignition::math::Vector3d world_b(
        p.X() - px, p.Y() - py, this->line_z_);
    const ignition::math::Vector3d world_c(
        p.X(), p.Y(), this->line_z_);
    const ignition::math::Vector3d world_d(
        p.X() + 0.5 * px, p.Y() + 0.5 * py, this->line_z_);
    const ignition::math::Vector3d world_e(
        p.X() - 0.5 * px, p.Y() - 0.5 * py, this->line_z_);

    // Transformacia world -> lokalny frame rodicovskeho visualu:
    //   world = parentPos + parentRot * (parentScale (.) local)
    //   => local = parentScale^-1 (.) ( parentRot^-1 * (world - parentPos) )
    // Tym sa kompenzuje lubovolna poza aj scale rodica (napr. plane ~50).
    const ignition::math::Pose3d   wp    = this->visual_->WorldPose();
    const ignition::math::Vector3d scale = this->visual_->DerivedScale();

    auto toLocal = [&](const ignition::math::Vector3d& w) {
      ignition::math::Vector3d r = wp.Rot().Inverse() * (w - wp.Pos());
      r.X() /= (scale.X() != 0.0 ? scale.X() : 1.0);
      r.Y() /= (scale.Y() != 0.0 ? scale.Y() : 1.0);
      r.Z() /= (scale.Z() != 0.0 ? scale.Z() : 1.0);
      return r;
    };

    this->line_a_->AddPoint(toLocal(world_a));
    this->line_b_->AddPoint(toLocal(world_b));
    this->line_c_->AddPoint(toLocal(world_c));
    this->line_d_->AddPoint(toLocal(world_d));
    this->line_e_->AddPoint(toLocal(world_e));

    this->last_ = p;
    this->has_last_ = true;
  }

  rendering::VisualPtr      visual_;
  rendering::DynamicLines*  line_a_ = nullptr;
  rendering::DynamicLines*  line_b_ = nullptr;
  rendering::DynamicLines*  line_c_ = nullptr;
  rendering::DynamicLines*  line_d_ = nullptr;
  rendering::DynamicLines*  line_e_ = nullptr;
  event::ConnectionPtr      update_conn_;

  std::string               robot_name_;
  double                    min_dist_    = 0.05;
  double                    line_z_      = 0.05;
  double                    line_offset_ = 0.025;

  ignition::math::Vector3d  last_;
  bool                      has_last_ = false;
};

GZ_REGISTER_VISUAL_PLUGIN(PathTrailVisualPlugin)

}  // namespace gazebo
