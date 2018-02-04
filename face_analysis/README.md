# users_analysis

Post-mortem users analysis from people perception and interactions collected data

## requeriments

* Monogdb dataset:

  * db : message_store_y4
  * collection: people_perception
  * collection: info_terminal_active_screen

* Videos recorded:

  * Folders: head_camera_2017-01, head_camera_2017-02, head_camera_2017-03

* For face analysis:

  * ROS module: ms_face_api

## Scripts:

### extract_video_data.py

The videos timestamp doesn’t correspond to the timestamp of database collections ( specially in interaction camera videos). To try to fix that issue this script request by wx interface the introduction of some video timestamp and create the file `info_videos_<namefolder>.json`. This file is used in future scripts to correspond the database timestamp with the correct video frame for Opencv analysis.

* Input: 
  * video folders (head_camera_2017-01, head_camera_2017-02, head_camera_2017-03)
  * path to videos folder (/.../videos)

* Parameters: 
  * `--videospath= ./data/videos`
  * `--videofolders= head_camera_2017-01, head_camera_2017-02, head_camera_2017-03`

* Output: 
  * JSON file: info_videos_<namefolder>.json




### people_perception2person_trajectory.py

`db('message_store_y4').getCollection('people_perception')` contents all the people detections (18926643 entries) and for each entry of `db('message_store_y4').getCollection('people_perception')` can be more than one person detected (uuid).This script creates a new collection `db('users_analysis').getCollection('trajectory_person')` filtering data from `db('message_store_y4').getCollection('people_perception')` by uuid. In this new collection each entry corresponds to each person (uuid) and contents the complete trajectory of the person.

* Input: 
  * The analysis is only realised on timestamps when there are videos availables (head_camera_2017-01, head_camera_2017-02, head_camera_2017-03)
  * filtered by:
  * total time person detected [secs]:  2.0 <= total_time <= 600.0
  * The robot is not navigating :`{'current_edge':'none'}`

* Parameters: 
  * `--datapath= ./data`
  * `--videofolders= head_camera_2017-01, head_camera_2017-02, head_camera_2017-03`
  * `--advancedfilter= false`

* Output: 
  * `db('users_analysis').getCollection('trajectory_person')`:
	```
		{
    "_id" : ObjectId("59dfbdcc00a06f26e8cefbff"),
    "init_timestamp" : 1483428361,
    "end_timestamp" : 1483428364,
    "total_time" : 3.0,
    "uuid" : "4ebb8025-bd71-5990-8ec8-984c40146b9e",
    "waypoint" : "ChargingPoint",
    "robot_pose" : {
        "position" : {
            "y" : -0.209205163502864,
            "x" : 0.144381121995913,
            "z" : 3.46944695195361e-18
        },
        "orientation" : {
            "y" : 0.0,
            "x" : 0.0,
            "z" : -0.0124239559554891,
            "w" : 0.999922819680807
        }
    },
    "person_poses" : [ 
        {
            "timestamp" : 1483428361,
            "angle" : -2.99229956071332,
            "distance" : 1.99156590691712
        },
        {
            "timestamp" : 1483428361,
            "angle" : -2.99229956071332,
            "distance" : 1.99156590691712
        }
    ]
	}


```
#### Advanced filter peson trajectory  (people_perception2person_trajectory.py)

It creates a new collection `db('users_analysis').getCollection('trajectory_person_filtered')` from `db('users_analysis').getCollection('trajectory_person')` to remove `false positives`, uuids which during the whole detection theirs positions don’t change more than 0.1 meters.

* Parameters:
  * `--advancedfilter= true`

* Input: 
  * `db('users_analysis').getCollection('trajectory_person')`
	

* Output: 
  * `db('users_analysis').getCollection('trajectory_person_filtered')`:


* TODO: The analysis should have be done only for the waypoints where was allowed video recording:

	 nodes_video_and_infoterminal=['Cafeteria', 'ChargingPoint', 'Feuerloescher', 'Frisoer', 'Teekueche', 'WayPoint12', 'WayPoint35', 'WayPoint57', 'WayPoint8']

### active_engaged_analysis.py

It creates 2 new collections `db('users_analysis').getCollection('active_interactions')` and `db('users_analysis').getCollection('active_users')` from `db('message_store_y4').getCollection('interactions')`.
`db('message_store_y4').getCollection('interactions')` contents every change of info-terminal screen. For each interaction the corresponding video timestamp is analysed at the first time by a basic face detector based on OpenCv, also a time window of +/- 2 secs is analysed around the interaction timestamp to find a face detection. If there any detection, the frame is sent to Microsoft Face API to further analysis and the respond is saved in `db('users_analysis').getCollection('active_users')`

* Input: 
  * The analysis is only realised on timestamps when there are videos availables (head_camera_2017-01, head_camera_2017-02, head_camera_2017-03)
  * The module `ms_face_api` is launched in `autotraining_mode` to re-identify previous users, or adding new users automatically if they are not yet in the Microsoft database.

* Parameters: 
  * `--datapath= ./data`
  * `--videofolders= head_camera_2017-01, head_camera_2017-02, head_camera_2017-03`
	
* Output: 
  * `db('users_analysis').getCollection('active_interactions')`:
```
			{
					"_id" : ObjectId("59108de700a06f1aee01911b"),
					"faces" : [ 
						  "user-0", 
						  "user-1"
					],
					"group" : true,
					"media" : {
						  "videoname" : "interaction_camera_2016-11-11_10-52-16.avi",
						  "numframe" : 797,
						  "timestamp" : 1478858768
					},
					"screen" : "weather",
					"timestamp" : 1478858768,
					"type" : "active",
					"uuids" : [],
					"waypoint" : "Frisoer"
			}
```

		* `faces` contents an array of the users name detected. Ordered by biggest face size detected
		* `uuids` In next steps, possible uuids associated from `trajectory_person` 

  * `db('users_analysis').getCollection('active_users')`:

		{
				"_id" : ObjectId("59d3f0ed00a06f17f572e072"),
				"user-ID" : "user-49",
				"interaction" : [ 
				    {
				        "timestamp" : 1484145298,
				        "demographics" : {
				            "gender" : "male",
				            "age" : 51.2
				        },
				        "confidence" : -1.0,
				        "asociated_inter" : ObjectId("59108de800a06f1aee019dcc"),
				        "mood" : 0.292,
				        "n-users" : 1,
				        "interaction-time" : 0,
				        "appearance" : {
				            "hair" : {
				            },
				            "emotion" : {
				            },
				            "accessories" : [],
				            "facialhair" : {
				            },
				            "glasses" : "NoGlasses"
				        },
				        "waypoint" : "Feuerloescher",
				        "screen" : "weather",
				        "primary-user" : true
				    }
				]
		}

		* `primary-user` if there are more than one face detected. The biggest face is considered as primary-user

  * For each new user a folder `user-id` with the detections frames is created in `<datapath>/users_active_engaged'`


* The analysis should have be done only for the waypoints where was allowed video recording:

	 nodes_video_and_infoterminal=['Cafeteria', 'ChargingPoint', 'Feuerloescher', 'Frisoer', 'Teekueche', 'WayPoint12', 'WayPoint35', 'WayPoint57', 'WayPoint8']

### passive_engaged_analysis.py

3 different functions: 

* automatic possible passive interactions `--mode= auto` 
* manual corrected passive interactions `--mode= manual` 
* face analysis passive interactions `--mode= faceanalysis` 

* Parameters:
  * `--mode= manual` 
  * `--datapath= ./data`
  * `--videofolders= head_camera_2017-01, head_camera_2017-02, head_camera_2017-03`
	

#### automatic possible passive interactions

From `db('users_analysis').getCollection('trajectory_person')` this script try to find people who look at the robot with interest. For each person detection timestamps the video frame is analysed by OpenCV to try to detect faces only if the people detected pose is inside the range `(angle > 2.0 or angle < -2.0 )` (The touchscreen is located at 3.14 rad).  If a face is detected, the angular position of the face in the image is compared with the angular position of the people tracker. If the distance is minor than 0.2 radians this uuid is possible passive interaction.
As result creates the collection `db('users_analysis').getCollection('passive_interactions')`.


* Input: 
  * `db('users_analysis').getCollection('trajectory_person')`
  * people detected positions in the range `(angle > 2.0 or angle < -2.0 )rad` 
  * distance between face position and people detection position less than `0.2 rad`

	
* Output: 

  * `db('users_analysis').getCollection('passive_interactions')`

		```
			{
					"_id" : ObjectId("59cfea9900a06f0d7f698daa"),
					"faces" : [ 
							{'xmin':0,'ymin':0,'width':0,'height':0,'timestamp':1484145298},
							{'xmin':0,'ymin':0,'width':0,'height':0,'timestamp':1484145298}
					],
					"interaction-type" : "passive-interaction",
					"screen" : "photos",
					"uuid" : "000a3ca6-aa3d-528b-a826-02a209f54791",
					"waypoint" : "ChargingPoint"
			}
		```

#### manual corrected passive interactions

It launchs a interface to classify manually the possible passive interactions from `db('users_analysis').getCollection('passive_interactions')`. For each `uuid` associated to each passive interaction the interface shows the video recorded. The interaction types are:

* non-person: it's a false positive detection
* non-interaction: There is person on the scene but it's not really a passive interaction
* passive-interaction: Any person look at the robot clearly. In this case, must stop the video on the frame which the face user has a good position for future analysis and select the region of the face.


* Input:
 
  * `db('users_analysis').getCollection('passive_interactions')`

* Output:
 	 Add  `interaction-type-labeled` and update `"faces" : [{'xmin':0,'ymin':0,'width':0,'height':0,'timestamp':1484145298}]`

		```
			{
					"_id" : ObjectId("59cfea9900a06f0d7f698daa"),
					"faces" : [ 
							{'xmin':0,'ymin':0,'width':0,'height':0,'timestamp':1484145298,"user-ID" : ""}
					],
					"interaction-type" : "passive-interaction",
					"interaction-type-labeled" : "non-interaction",
					"screen" : "photos",
					"uuid" : "000a3ca6-aa3d-528b-a826-02a209f54791",
					"waypoint" : "ChargingPoint"
			}

  * For each `passive-interaction` a new folder with the detection frame is created in `'<datapath>/output_data/passive_interactions_uuids'`


#### face analysis passive interactions

From `db('users_analysis').getCollection('passive_interactions')` for each `interaction-type-labeled =passive-interaction` the corresponding frame is sent to Microsoft Face API to further analysis and the respond is saved in `db('users_analysis').getCollection('passive_users')`

* Input:
 
  * `db('users_analysis').getCollection('passive_interactions')`

* Output:

  * `db('users_analysis').getCollection('passive_users')`:

		{
				"_id" : ObjectId("59d3f0ed00a06f17f572e072"),
				"user-ID" : "user-49",
				"interaction" : [ 
				    {
				        "timestamp" : 1484145298,
				        "demographics" : {
				            "gender" : "male",
				            "age" : 51.2
				        },
				        "confidence" : -1.0,
				        "asociated_inter" : ObjectId("59108de800a06f1aee019dcc"),
				        "mood" : 0.292,
				        "n-users" : 1,
				        "interaction-time" : 0,
				        "appearance" : {
				            "hair" : {
				            },
				            "emotion" : {
				            },
				            "accessories" : [],
				            "facialhair" : {
				            },
				            "glasses" : "NoGlasses"
				        },
				        "waypoint" : "Feuerloescher",
				        "screen" : "weather",
				        "primary-user" : false
							
				    }
				]
		}
		* `confidence` show the confidence on to be really the same user. -1 means it's the trainning face.
		* `primary-user` is always true in this case

  * Update `db('users_analysis').getCollection('passive_interactions')`

	  If there is face detection the value `user-ID` in `"faces" : [{"timestamp" : 1483638376,"user-ID" : ""}]` change.

* TODO: 
	passive interaction --> active interaction

### users2frongo.py


* Input:
  * `db('users_analysis').getCollection('active_users')` 
  * `db('users_analysis').getCollection('passive_users')`


* Output:
  * `db('users_analysis').getCollection('final_users')` 


