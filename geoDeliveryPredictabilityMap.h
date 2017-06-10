
/*
 * DeliveryPredictabilityMap.h
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
#include "Configuration.h"
#ifndef GEODELIVERYPREDICTABILITYMAP_H_
#define GEODELIVERYPREDICTABILITYMAP_H_
#include "routing/NeighborDataset.h"
#include "routing/NodeHandshake.h"
#include "routing/NodeHandshakeEvent.h"
#include <ibrdtn/data/EID.h>
#include <ibrcommon/thread/Mutex.h>
#include <map>
#include <GPSClient/gps_read.h>
using namespace dtn::GPSClient;
namespace dtn
{
	namespace routing
	{
		/*!
		 * \brief This class keeps track of the predictablities to see a specific EID
		 *
		 * This class can be used as a map from EID to float.
		 * Also, it can be serialized as a NodeHandshakeItem to be exchanged with neighbors.
		 */
		class geoDeliveryPredictabilityMap : public NeighborDataSetImpl, public NodeHandshakeItem,public dtn::GPSClient::gps_read, public ibrcommon::Mutex {
		public:
			static const dtn::data::Number identifier;

			geoDeliveryPredictabilityMap();
			geoDeliveryPredictabilityMap(const size_t &time_unit, const float &beta, const float &gamma);
			virtual ~geoDeliveryPredictabilityMap();

			virtual const dtn::data::Number& getIdentifier() const; ///< \see NodeHandshakeItem::getIdentifier
			virtual dtn::data::Length getLength() const; ///< \see NodeHandshakeItem::getLength
			virtual std::ostream& serialize(std::ostream& stream) const; ///< \see NodeHandshakeItem::serialize
			virtual std::istream& deserialize(std::istream& stream); ///< \see NodeHandshakeItem::deserialize

			class ValueNotFoundException : public ibrcommon::Exception
			{
			public:
				ValueNotFoundException()
				: ibrcommon::Exception("The requested value is not available.") { };

				virtual ~ValueNotFoundException() throw () { };
			};

			val17 get(const dtn::data::EID &neighbor) const throw (ValueNotFoundException);
			void set(const dtn::data::EID &neighbor, val17);
			void clear();
			size_t size() const;

			/*!
			 * Updates the DeliveryPredictabilityMap with one received by a neighbor.
			 * \param dpm the DeliveryPredictabilityMap received from the neighbor
			 * \warning The _deliveryPredictabilityMap has to be locked before calling this function
			 */
			void update(const dtn::data::EID &origin, const geoDeliveryPredictabilityMap &dpm, const float &p_encounter_first);

			/*!
			 * Age all entries in the DeliveryPredictabilityMap.
			 * \warning The _deliveryPredictabilityMap has to be locked before calling this function
			 */
			void age(const float &p_first_threshold);

			/**
			 * Print out the content as readable text.
			 */
			void toString(std::ostream &stream) const;

			friend std::ostream& operator<<(std::ostream& stream, const geoDeliveryPredictabilityMap& map);

			/**
			 * stores the map and time-stamp of last aging to a stream
			 */
			void store(std::ostream &output) const;

			/**
			 * restore the map and time-stamp of last aging from a stream
			 */
			void restore(std::istream &input);

		private:

			typedef std::map<dtn::data::EID, struct  val17> predictmap;//set_value> predictmap;
			predictmap _predictmap;

			float _beta; ///< Weight of the transitive property of geoprophet.
			float _gamma; ///< Determines how quickly predictabilities age.

			dtn::data::Timestamp _lastAgingTime; ///< Timestamp when the map has been aged the last time.
			size_t _time_unit; ///< time unit to be used in the network
			gps_read   _gps_read;
			//const dtn::daemon::Configuration &_config ;

struct val15{
float val1;
float val2;
float val3;
float val4;
float val5;
float val6;
float val7;}values5;

struct val14{
	float val1;
	float val2;
	float val3;
	float val4;
	float val5;
	float val6;
	float val7;}values4;


		};
	} /* namespace routing */
} /* namespace dtn */
#endif /* DELIVERYPREDICTABILITYMAP_H_ */
