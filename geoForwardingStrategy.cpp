
/*
 * geoForwardingStrategy.cpp
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

#include "routing/geoprophet/geoForwardingStrategy.h"
#include "routing/geoprophet/geoProphetRoutingExtension.h"
#include "routing/geoprophet/geoDeliveryPredictabilityMap.h"

namespace dtn
{
	namespace routing
	{
	geoForwardingStrategy::geoForwardingStrategy()
		 : _geoprophet_router(NULL)
		{}

	geoForwardingStrategy::~geoForwardingStrategy()
		{}

		bool geoForwardingStrategy::neighborDPIsGreater(const geoDeliveryPredictabilityMap& neighbor_dpm, const dtn::data::EID& destination) const
		{
			const dtn::data::EID destnode = destination.getNode();

			try {
				float local_pv = 0.0f;
				val17 values7,values8;
				val12 values2,values3;
				val11 values1;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
				//p_ab=1/(_gps_read.calculate_distance(_gps_read.gps_values(values1),values2)+_gps_read.calculate_speed( _gps_read.gps_values(values1), values2)+1);
				////////////////////////////////////////////////////////////////////////////////////////////
				// get the local value
				{
					ibrcommon::MutexLock dpm_lock(_geoprophet_router->_deliveryPredictabilityMap);
					const geoDeliveryPredictabilityMap& dp_map = _geoprophet_router->_deliveryPredictabilityMap;
					//local_pv = dp_map.get(destnode);
					values7=dp_map.get(destnode);
					values2.val1=values7.val1;
					values2.val2=values7.val2;
					values2.val3=values7.val3;
					values2.val4=values7.val4;
					values2.val5=values7.val5;
					values2.val6=values7.val6;
					values2.val7=values7.val7;

				}

				try {
					// retrieve the value from the DeliveryPredictabilityMap of the neighbor
					//float foreign_pv = neighbor_dpm.get(destnode);
					float foreign_pv=0.0f;
					values8=neighbor_dpm.get(destnode);
					values3.val1=values8.val1;
					values3.val2=values8.val2;
					values3.val3=values8.val3;
					values3.val4=values8.val4;
					values3.val5=values8.val5;
					values3.val6=values8.val6;
					values3.val7=values8.val7;
//foreign_pv=1/(_gps_read.calculate_distance(_gps_read.gps_values(values1),values3)+_gps_read.calculate_speed( _gps_read.gps_values(values1), values3)+1);
//local_pv=1/(_gps_read.calculate_distance(_gps_read.gps_values(values1),values2)+_gps_read.calculate_speed( _gps_read.gps_values(values1), values2)+1);
foreign_pv=1/(const_cast<geoForwardingStrategy*>(this)->_gps_read.calculate_distance(const_cast<geoForwardingStrategy*>(this)->_gps_read.gps_values(values1),values3)+const_cast<geoForwardingStrategy*>(this)->_gps_read.calculate_speed( const_cast<geoForwardingStrategy*>(this)->_gps_read.gps_values(values1), values3)+1);
local_pv=1/(const_cast<geoForwardingStrategy*>(this)->_gps_read.calculate_distance(const_cast<geoForwardingStrategy*>(this)->_gps_read.gps_values(values1),values2)+const_cast<geoForwardingStrategy*>(this)->_gps_read.calculate_speed( const_cast<geoForwardingStrategy*>(this)->_gps_read.gps_values(values1), values2)+1);
//foreign_pv=1/(const_cast<ForwardingStrategy*>(this)->(_gps_read.calculate_distance(const_cast<ForwardingStrategy*>(this)->(_gps_read.gps_values(values1)),values3))+_const_cast<ForwardingStrategy*>(this)->(_gps_read.calculate_speed(const_cast<ForwardingStrategy*>(this)->(_gps_read.gps_values(values1)), values3))+1);



return (foreign_pv > local_pv);



				} catch (const geoDeliveryPredictabilityMap::ValueNotFoundException&) {
					// if the foreign router has no entry for the destination
					// then compare the local value with a fictitious initial value
					return (_geoprophet_router->_p_first_threshold > local_pv);

				}
			} catch (const geoDeliveryPredictabilityMap::ValueNotFoundException&) {
				// always forward if the destination is not in our own predictability map
			}

			return false;
		}

		void geoForwardingStrategy::setProphetRouter(geoProphetRoutingExtension *router)
		{
			_geoprophet_router = router;
		}
	} /* namespace routing */
} /* namespace dtn */
