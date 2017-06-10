/*
 * geoForwardingStrategy.h
 *
 * Copyright (C) 2015 TNO
 *
 * Written-by: Tadewos Somano <Tadewos.SomanoEwalo@tno.nl,tadewos85@gmail.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#ifndef GEOFORWARDINGSTRATEGY_H_
#define GEOFORWARDINGSTRATEGY_H_
#include "routing/geoprophet/geoDeliveryPredictabilityMap.h"
#include <ibrdtn/data/MetaBundle.h>
#include <ibrdtn/data/EID.h>
#include "routing/NodeHandshake.h"
#include <GPSClient/gps_read.h>
#include"Configuration.h"
using namespace dtn::GPSClient;
namespace dtn
{
	namespace routing
	{
		class geoProphetRoutingExtension;

		/*!
		 * \brief This class is a abstract base class for all prophet forwarding strategies.
		 */
		class geoForwardingStrategy:public dtn::GPSClient::gps_read
		{
		public:
			geoForwardingStrategy();
			virtual ~geoForwardingStrategy() = 0;
			/*!
			 * The prophetRoutingExtension calls this function for every bundle that can be forwarded to a neighbor
			 * and forwards it depending on the return value.
			 * \param neighbor the neighbor to forward to
			 * \param bundle the bundle that can be forwarded
			 * \param prophet_router Reference to the ProphetRoutingExtension to access its parameters
			 * \return true if the bundle should be forwarded
			 */
			virtual bool shallForward(const geoDeliveryPredictabilityMap& neighbor_dpm, const dtn::data::MetaBundle& bundle) const = 0; //const=0;
			/*!
			 * checks if the deliveryPredictability of the neighbor is higher than that of the destination of the bundle.
			 */
			bool neighborDPIsGreater(const geoDeliveryPredictabilityMap& neighbor_dpm, const dtn::data::EID& destination) const;

			/**
			 * Set back-reference to the prophet router
			 */
			void setProphetRouter(geoProphetRoutingExtension *router);

		protected:
			geoProphetRoutingExtension *_geoprophet_router;
			//const dtn::daemon::Configuration &_config ;
		private:
			gps_read   _gps_read;
		};
	} /* namespace routing */
} /* namespace dtn */
#endif /* FORWARDINGSTRATEGY_H_ */
