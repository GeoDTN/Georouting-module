/*
 * geoProphetRoutingExtension.h
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
#ifndef GEOProphetRoutingExtension_H_
#define GEOProphetRoutingExtension_H_
#include<cstring>
#include "routing/geoprophet/geoDeliveryPredictabilityMap.h"
#include "routing/geoprophet/geoForwardingStrategy.h"
#include "routing/geoprophet/geoAcknowledgementSet.h"
#include "routing/RoutingExtension.h"
#include "core/EventReceiver.h"
#include "routing/NodeHandshakeEvent.h"
#include "core/TimeEvent.h"
#include "core/BundlePurgeEvent.h"
#include <ibrcommon/thread/Mutex.h>
#include <ibrcommon/thread/Queue.h>
#include <ibrcommon/thread/ThreadsafeReference.h>
#include <map>
#include <list>
#include "routing/NodeHandshake.h"
#include <GPSClient/gps_read.h>
#include"Configuration.h"
using namespace dtn::GPSClient;
namespace dtn
{
	namespace routing
	{
		/*!
		 * \brief Routing extension for PRoPHET routing
		 *
		 * PRoPHET keeps track of predictabilities to see specific other nodes.
		 * i.e. Nodes that are seen often will have a high predictabilites attached
		 * and forwarding decisions can be made based on these predictabilites.
		 * In addition, these predictabilites are updated transitively by exchanging
		 * predictabilityMaps with neighbors.
		 * For a detailed description of the protocol, see draft-irtf-dtnrg-prophet-09
		 */
		class geoProphetRoutingExtension : public RoutingExtension, public ibrcommon::JoinableThread,
			public dtn::core::EventReceiver<dtn::routing::NodeHandshakeEvent>,
			public dtn::core::EventReceiver<dtn::core::TimeEvent>,
			public dtn::core::EventReceiver<dtn::core::BundlePurgeEvent>,
			public dtn::core::EventReceiver<dtn::GPSClient::GPSEvent>,
			public  dtn::GPSClient::gps_read,public dtn::GPSClient::GPSEvent
		{
			friend class geoForwardingStrategy;
			static const std::string TAG;

		public:
			geoProphetRoutingExtension(geoForwardingStrategy *strategy,
						float p_first_threshold,float beta,float gamma,
						size_t time_unit, size_t i_typ,
						dtn::data::Timestamp next_exchange_timeout);
			virtual ~geoProphetRoutingExtension();

			virtual const std::string getTag() const throw ();

			/* virtual methods from BaseRouter::Extension */
			virtual void requestHandshake(const dtn::data::EID&, NodeHandshake&) const; ///< \see BaseRouter::Extension::requestHandshake
			virtual void responseHandshake(const dtn::data::EID&, const NodeHandshake&, NodeHandshake&); ///< \see BaseRouter::Extension::responseHandshake
			virtual void processHandshake(const dtn::data::EID&, NodeHandshake&); ///< \see BaseRouter::Extension::processHandshake
			virtual void componentUp() throw ();
			virtual void componentDown() throw ();

			virtual void raiseEvent(const dtn::routing::NodeHandshakeEvent &evt) throw ();
			virtual void raiseEvent(const dtn::core::TimeEvent &evt) throw ();
			virtual void raiseEvent(const dtn::core::BundlePurgeEvent &evt) throw ();
	        virtual void raiseEvent(const dtn::GPSClient::GPSEvent &GEvent) throw ();
			virtual void eventDataChanged(const dtn::data::EID &peer) throw ();

			virtual void eventTransferCompleted(const dtn::data::EID &peer, const dtn::data::MetaBundle &meta) throw ();

			virtual void eventBundleQueued(const dtn::data::EID &peer, const dtn::data::MetaBundle &meta) throw ();

			/*!
			 * Returns a threadsafe reference to the DeliveryPredictabilityMap. I.e. the corresponding
			 * Mutex is locked while this object exists.
			 * In addition, the map is aged before it is returned.
			 */
			ibrcommon::ThreadsafeReference<geoDeliveryPredictabilityMap> getDeliveryPredictabilityMap();
			/*!
			 * Returns a threadsafe reference to the DeliveryPredictabilityMap. I.e. the corresponding
			 * Mutex is locked while this object exists.
			 * This const version does not age the map.
			 */
			ibrcommon::ThreadsafeReference<const geoDeliveryPredictabilityMap> getDeliveryPredictabilityMap() const;

			/*!
			 * Returns a threadsafe reference to the AcknowledgementSet. I.e. the corresponding
			 * Mutex is locked while this object exists.
			 */
			ibrcommon::ThreadsafeReference<const geoAcknowledgementSet> getAcknowledgementSet() const;
		protected:
			virtual void run() throw ();
			void __cancellation() throw ();
		private:
			/*!
			 * Updates the DeliveryPredictabilityMap in the event that a neighbor has been encountered.
			 * \warning The _deliveryPredictabilityMap has to be locked before calling this function
			 */

			//void updateNeighbor(const dtn::data::EID& neighbor, const DeliveryPredictabilityMap& neighbor_dp_map);

			/*!
			 * Age all entries in the DeliveryPredictabilityMap.
			 * \warning The _deliveryPredictabilityMap has to be locked before calling this function
			 */
			void age();

			/**
			 * stores all persistent data to a file
			 */
			void store(const ibrcommon::File &target);

			/**
			 * restore all persistent data from a file
			 */
			void restore(const ibrcommon::File &source);

			geoDeliveryPredictabilityMap _deliveryPredictabilityMap;
			geoForwardingStrategy *_forwardingStrategy;
			geoAcknowledgementSet _acknowledgementSet;

			ibrcommon::Mutex _next_exchange_mutex; ///< Mutex for the _next_exchange_timestamp.
			dtn::data::Timestamp _next_exchange_timeout; ///< Interval in seconds how often Handshakes should be executed on longer connections.
			dtn::data::Timestamp _next_exchange_timestamp; ///< Unix timestamp, when the next handshake is due.

			/*!
			 * Calculates the p_encounter that rises linearly with the time since the encounter, up to p_encounter_max.
			 */
			//float p_encounter(const dtn::data::EID &neighbor) const;

			float _p_encounter_first; ///< Value to which the predictability is set on the first encounter.
			float _p_first_threshold; ///< If the predictability falls below this value, it is erased from the map and _p_encounter_first will be used on the next encounter.


			size_t _i_typ; ///< time interval that is characteristic for the network

			typedef std::map<dtn::data::EID, dtn::data::Timestamp> age_map;
			age_map _ageMap; ///< map with time for each neighbor, when the last encounter happened

			ibrcommon::File _persistent_file;
			gps_read   _gps_read;///< This file is used to store persistent routing data

			class Task
			{
			public:
				virtual ~Task() {};
				virtual std::string toString() const = 0;
			};

			class SearchNextBundleTask : public Task
			{
			public:
				SearchNextBundleTask(const dtn::data::EID &eid);
				virtual ~SearchNextBundleTask();

				virtual std::string toString() const;
				const dtn::data::EID eid;
			};

			class NextExchangeTask : public Task
			{
			public:
				NextExchangeTask();
				virtual ~NextExchangeTask();

				virtual std::string toString() const;
			};

			ibrcommon::Queue<Task* > _taskqueue;

		public:
			/*!
			 * \brief The GRTR forwarding strategy.
			 * Using this strategy, packets are forwarding, if the neighbor
			 * has a higher predictability than the destination.
			 */
			class GRTR_Strategy : public geoForwardingStrategy
			{
			public:
				explicit GRTR_Strategy();
				virtual ~GRTR_Strategy();
				virtual bool shallForward(const geoDeliveryPredictabilityMap& neighbor_dpm, const dtn::data::MetaBundle& bundle) const;//const;
			};

			/*!
			 * \brief The GTMX forwarding strategy.
			 * Using this strategy, packets are forwarding, if the neighbor
			 * has a higher predictability than the destination, but at most
			 * NF_max times.
			 */
			class GTMX_Strategy : public geoForwardingStrategy
			{
			public:
				explicit GTMX_Strategy(unsigned int NF_max);
				virtual ~GTMX_Strategy();
				virtual bool shallForward(const geoDeliveryPredictabilityMap& neighbor_dpm, const dtn::data::MetaBundle& bundle) const;//const;

				void addForward(const dtn::data::BundleID &id);

			private:
				unsigned int _NF_max; ///< Number how often a bundle should be forwarded at most.

				typedef std::map<dtn::data::BundleID, unsigned int> nf_map;
				nf_map _NF_map;
				///< Map where the number of forwards are saved.
				gps_read _gps_read;
                                 
			};
		};
	} // namespace routing
} // namespace dtn

#endif // geoProphetRoutingExtension_H_



















