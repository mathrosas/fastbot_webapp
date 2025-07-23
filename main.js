var vueApp = new Vue({
    el: '#vueApp',
    // storing the state of the page
    data: {
        connected: false,
        ros: null,
        logs: [],
        loading: false,
        rosbridge_address: 'wss://i-064b7f80e1e032cd9.robotigniteacademy.com/31bec39d-39db-47c2-9ffb-4140d22c3cfa/rosbridge/',
        port: '9090',
        menu_title: 'FastBot Web Control Dashboard',
        // 3D stuff
        viewer: null,
        tfClient: null,
        urdfClient: null,
        mapViewer: null,
        mapClient: null,
        dragging: false,
        x: 'no',
        y: 'no',
        dragCircleStyle: {
            margin: '0px',
            top: '0px',
            left: '0px',
            display: 'none',
            width: '75px',
            height: '75px',
        },
        // joystick valules
        joystick: {
            vertical: 0,
            horizontal: 0,
        },
        // publisher
        pubInterval: null,
        odomTopic: null,
        position: { x:0, y:0, z:0 }
    },
    // helper methods to connect to ROS
    methods: {
        connect: function() {
            this.loading = true
            this.ros = new ROSLIB.Ros({
                url: this.rosbridge_address,
                groovyCompatibility: false
            })
            this.ros.on('connection', () => {
                this.logs.unshift((new Date()).toTimeString() + ' - Connected!')
                this.connected = true
                this.loading = false
                this.setup3DViewer()
                this.setupMapViewer()
                this.pubInterval = setInterval(this.publish, 100)

                // Subscribe to odometry
                this.odomTopic = new ROSLIB.Topic({
                    ros: this.ros,
                    name: '/fastbot_1/odom',
                    messageType: 'nav_msgs/Odometry'
                })
                
                this.odomTopic.subscribe((message) => {
                    this.position = message.pose.pose.position
                })
            })
            this.ros.on('error', (error) => {
                this.logs.unshift((new Date()).toTimeString() + ` - Error: ${error}`)
            })
            this.ros.on('close', () => {
                this.logs.unshift((new Date()).toTimeString() + ' - Disconnected!')
                this.connected = false
                this.loading = false
                this.unset3DViewer()
                clearInterval(this.pubInterval)
                if (this.odomTopic) this.odomTopic.unsubscribe()
            })
        },
        setup3DViewer() {
            // 1) figure out how big the container really is
            const viewerDiv = document.getElementById('div3DViewer')
            const w = viewerDiv.clientWidth
            const h = viewerDiv.clientHeight

            // 2) use those dimensions instead of a magic “400×300”
            this.viewer = new ROS3D.Viewer({
                background: '#cccccc',
                divID:     'div3DViewer',
                width:     w,
                height:    h,
                antialias: true,
                fixedFrame:'fastbot_1_odom'
            })

            // Add a grid.
            this.viewer.addObject(new ROS3D.Grid({
                color:'#0181c4',
                cellSize: 0.5,
                num_cells: 20
            }))

            // Setup a client to listen to TFs.
            this.tfClient = new ROSLIB.TFClient({
                ros: this.ros,
                angularThres: 0.01,
                transThres: 0.01,
                rate: 10.0,
                fixedFrame: 'fastbot_1_base_link'
            })

            // Setup the URDF client.
            this.urdfClient = new ROS3D.UrdfClient({
                ros: this.ros,
                param: '/fastbot_1_robot_state_publisher:robot_description',
                tfClient: this.tfClient,
                // We use "path: location.origin + location.pathname"
                // instead of "path: window.location.href" to remove query params,
                // otherwise the assets fail to load
                path: location.origin + location.pathname,
                rootObject: this.viewer.scene,
                loader: ROS3D.COLLADA_LOADER_2
            })
        },
        unset3DViewer() {
            document.getElementById('div3DViewer').innerHTML = ''
        },
        setupMapViewer() {
            // 1) create the 2D viewer
            const mapDiv = document.getElementById('divMapViewer')
            this.mapViewer = new ROS2D.Viewer({
                divID:  'divMapViewer',
                width:  mapDiv.clientWidth,
                height: mapDiv.clientHeight
            })

            // 2) subscribe to the /map topic and draw it, continuously
            this.mapClient = new ROS2D.OccupancyGridClient({
                ros:        this.ros,
                rootObject: this.mapViewer.scene,
                topic:      '/map',
                continuous: true,
            })

            // 3) once the grid arrives or updates, scale AND SHIFT to center
            this.mapClient.on('change', () => {
                const grid = this.mapClient.currentGrid;
                this.mapViewer.scaleToDimensions(grid.width, grid.height);
                // recenter the map so (0,0) is in the middle of the canvas
                this.mapViewer.shift(
                grid.pose.position.x,
                grid.pose.position.y
                )
            })
        },
        // — Joystick drag handlers —
        publish: function() {
            let topic = new ROSLIB.Topic({
                ros: this.ros,
                name: '/fastbot_1/cmd_vel',
                messageType: 'geometry_msgs/Twist'
            })
            let message = new ROSLIB.Message({
                linear: { x: this.joystick.vertical, y: 0, z: 0, },
                angular: { x: 0, y: 0, z: this.joystick.horizontal, },
            })
            topic.publish(message)
        },
        disconnect: function() {
            this.ros.close()
        },
        sendCommand: function() {
            let topic = new ROSLIB.Topic({
                ros: this.ros,
                name: '/fastbot_1/cmd_vel',
                messageType: 'geometry_msgs/Twist'
            })
            let message = new ROSLIB.Message({
                linear: { x: 0.2, y: 0, z: 0, },
                angular: { x: 0, y: 0, z: 0.5, },
            })
            topic.publish(message)
        },
        startDrag() {
            this.dragging = true
            this.x = this.y = 0
        },
        stopDrag() {
            this.dragging = false
            this.x = this.y = 'no'
            this.dragCircleStyle.display = 'none'
            this.resetJoystickVals()
        },
        doDrag(event) {
            if (this.dragging) {
                this.x = event.offsetX
                this.y = event.offsetY
                let ref = document.getElementById('dragstartzone')
                this.dragCircleStyle.display = 'inline-block'

                let minTop = ref.offsetTop - parseInt(this.dragCircleStyle.height) / 2
                let maxTop = minTop + 200
                let top = this.y + minTop
                this.dragCircleStyle.top = `${top}px`

                let minLeft = ref.offsetLeft - parseInt(this.dragCircleStyle.width) / 2
                let maxLeft = minLeft + 200
                let left = this.x + minLeft
                this.dragCircleStyle.left = `${left}px`

                this.setJoystickVals()
            }
        },
        setJoystickVals() {
            this.joystick.vertical = -1 * ((this.y / 200) - 0.5)
            this.joystick.horizontal = +1 * ((this.x / 200) - 0.5)
        },
        resetJoystickVals() {
            this.joystick.vertical = 0
            this.joystick.horizontal = 0
        },
    },
    mounted() {
        // page is ready
        window.addEventListener('mouseup', this.stopDrag)
        // window.addEventListener('touchend', this.stopDrag)
    },
})
