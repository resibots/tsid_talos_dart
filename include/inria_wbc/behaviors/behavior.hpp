#ifndef IWBC_BEHAVIOR_HPP
#define IWBC_BEHAVIOR_HPP

#include <inria_wbc/controllers/talos_base_controller.hpp>

namespace inria_wbc {
    namespace behaviors {
        class Behavior {
        public:
            Behavior(const std::shared_ptr<inria_wbc::controllers::TalosBaseController>& controller) : controller_(controller) {}
            virtual ~Behavior() {}
            // when we have no FT data to provide (might not always work!)
            virtual bool update() { return update(controllers::SensorData()); }
            // with FT data
            virtual bool update(const controllers::SensorData&) = 0;

            virtual std::shared_ptr<inria_wbc::controllers::TalosBaseController> controller() { return controller_; };
            virtual std::shared_ptr<const inria_wbc::controllers::TalosBaseController> controller() const { return controller_; };

        protected:
            std::shared_ptr<inria_wbc::controllers::TalosBaseController> controller_;
        };
    } // namespace behaviors
} // namespace inria_wbc
#endif
