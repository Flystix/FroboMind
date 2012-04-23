/*
 *  Ground Station for CityFlyer CCNY project
 *  Copyright (C) 2010, CCNY Robotics Lab
 *  Gautier Dumonteil <gautier.dumonteil@gmail.com>
 *  http://robotics.ccny.cuny.edu
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file ground_station.cpp 
 * @brief Program that link ROS with Gtk
 * @author Gautier Dumonteil <gautier.dumonteil@gmail.com>
 * @version 0.1
 * @date 06/06/2010
 *
 * Ground Station for CityFlyer CCNY project
 * Copyright (C) 2010, CCNY Robotics Lab
 * http://robotics.ccny.cuny.edu
 *  
 */

#include <ground_station/ground_station.h>
//#include <fmMsgs/teleAir2Ground.h>
#include <fmMsgs/airframeState.h>
#include <fmMsgs/sysState.h>
#include <fmMsgs/gps_state.h>
#include <unistd.h>

AppData *data;

void airframeStateCallback(const fmMsgs::airframeState::ConstPtr& msg);

void systemStateCallback(const fmMsgs::sysState::ConstPtr&);

void gpsStateCallback(const fmMsgs::gps_state::ConstPtr&);

/**
 * @fn void *startROS (void *user)
 * @brief ROS thread.
 * 
 * The main program wait until "ros_param_read" in order to allow the <br>
 * ROS params to be also the Gtk Window and Widgets params.
 * Then the ROS thread wait to the widgets creation before subcribing<br>
 * to any topics, avoid to call public widget function for a widget not<br>
 * yet created.
 */
void *startROS(void *user) {
	if (user != NULL) {
		struct arg *p_arg = (arg *) user;

		ros::init(p_arg->argc, p_arg->argv, "ground_station");
		ros::NodeHandle n;

		std::string local_path;
		std::string package_path = ros::package::getPath(ROS_PACKAGE_NAME);
		ros::NodeHandle np("~");
		XmlRpc::XmlRpcValue xml_marker_center;

		ROS_INFO ("Starting CityFlyer Ground Station");

		// -----------------------------------------------------------------
		// **** General window parameters
		if (!np.getParam("window_grayscale_color", data->grayscale_color))
			data->grayscale_color = false;
		ROS_DEBUG("\tWindow grayscale color: %d", data->grayscale_color);

		if (!np.getParam("window_radial_color", data->radial_color))
			data->radial_color = true;
		ROS_DEBUG("\tWindow radial color: %d", data->radial_color);

		np.param<int> ("telemetry_refresh_rate", data->telemetry_refresh_rate, 50);
		data->telemetry_refresh_rate = (int) round((float) (1000 / data->telemetry_refresh_rate));
		ROS_WARN("\tTelemetry refreshing every %d ms", data->telemetry_refresh_rate);

		// -----------------------------------------------------------------
		// **** Gauge1 parameters
		std::string gauge1_name, gauge1_unit, gauge1_color_strip, gauge1_sub_step;

		np.param("gauge1_name", gauge1_name, std::string("Gauge 1"));
		np.param("gauge1_unit", gauge1_unit, std::string("(unit)"));
		sprintf(data->gauge1_name_f,
		        "<big>%s</big>\n<span foreground=\"orange\"><i>(%s)</i></span>",
		        gauge1_name.c_str(), gauge1_unit.c_str());
		ROS_DEBUG ("\tGauge 1 name : %s", data->gauge1_name_f);

		if (!np.getParam("gauge1_start_value", data->gauge1_start_value))
			data->gauge1_start_value = 0;
		ROS_DEBUG ("\tGauge 1 start value: %d", data->gauge1_start_value);

		if (!np.getParam("gauge1_end_value", data->gauge1_end_value))
			data->gauge1_end_value = 100;
		ROS_DEBUG ("\tGauge 1 end value: %d", data->gauge1_end_value);

		if (!np.getParam("gauge1_initial_step", data->gauge1_initial_step))
			data->gauge1_initial_step = 10;
		ROS_DEBUG ("\tGauge 1 initial step value: %d", data->gauge1_initial_step);

		if (!np.getParam("gauge1_sub_step", gauge1_sub_step)) {
			data->gauge1_sub_step = 2;
		} else {
			size_t found = gauge1_sub_step.find_first_of("_");
			gauge1_sub_step[found] = '.';
			ROS_DEBUG ("\tGauge 1 sub step value: %s", gauge1_sub_step.c_str ());
			try {
				data->gauge1_sub_step = boost::lexical_cast<double>(gauge1_sub_step);
			} catch (const std::exception &) {
				data->gauge1_sub_step = 0; // **** Will cause a Gtk warning on the gauge
			}
		}

		if (!np.getParam("gauge1_drawing_step", data->gauge1_drawing_step))
			data->gauge1_drawing_step = 10;
		ROS_DEBUG ("\tGauge 1 drawing step value: %d", data->gauge1_drawing_step);

		// **** OPTIONAL
		np.param("gauge1_color_strip_order", gauge1_color_strip, std::string("YOR"));
		sprintf(data->gauge1_color_strip_order, "%s", gauge1_color_strip.c_str());
		np.getParam("gauge1_green_strip_start", data->gauge1_green_strip_start);
		np.getParam("gauge1_yellow_strip_start", data->gauge1_yellow_strip_start);
		np.getParam("gauge1_orange_strip_start", data->gauge1_orange_strip_start);
		np.getParam("gauge1_red_strip_start", data->gauge1_red_strip_start);

		// -----------------------------------------------------------------
		// **** Altimeter parameters
		if (!np.getParam("altimeter_unit_is_feet", data->altimeter_unit_is_feet))
			data->altimeter_unit_is_feet = true;
		ROS_DEBUG ("\tAltimeter unit is FEET: %d", data->altimeter_unit_is_feet);

		if (!np.getParam("altimeter_step_value", data->altimeter_step_value))
			data->altimeter_step_value = 100;
		ROS_DEBUG ("\tAltimeter step value: %d", data->altimeter_step_value);

		// -----------------------------------------------------------------
		// **** Bar Gauge parameters
		std::string widget_name;
		std::string name_bar_gauge1, name_bar_gauge2, name_bar_gauge3;
		std::string unit_bar_gauge1, unit_bar_gauge2, unit_bar_gauge3;

		np.param("bg_widget_name", widget_name, std::string("Randow Bar Gauges"));
		np.param<int> ("bg_bar_number", data->bar_number, 1);
		sprintf(data->widget_name, "<big>%s</big>", widget_name.c_str());

		ROS_DEBUG ("\tWidget name : %s", data->widget_name);
		ROS_DEBUG ("\tNum containser of bar gauge: %d", data->bar_number);

		// Bar gauge 1
		np.param<std::string> ("bg_name_bar_gauge1", name_bar_gauge1, std::string("BG1"));
		np.param<std::string> ("bg_unit_bar_gauge1", unit_bar_gauge1, std::string("[x]"));
		sprintf(data->name_bar_gauge1_f,
		        "<big>%s</big>\n<span foreground=\"orange\"><i>(%s)</i></span>",
		        name_bar_gauge1.c_str(), unit_bar_gauge1.c_str());
		np.param<int> ("bg_start_value_bar_1", data->start_value_bar_1, 100);
		np.param<int> ("bg_end_value_bar_1", data->end_value_bar_1, 0);
		np.param<int> ("bg_green_strip_start_1", data->green_strip_start_1, data->end_value_bar_1);
		np.param<int> ("bg_yellow_strip_start_1", data->yellow_strip_start_1,
		               data->start_value_bar_1);
		np.param<int> ("bg_red_strip_start_1", data->red_strip_start_1, data->start_value_bar_1);
		ROS_DEBUG ("\tBar Gauge 1 name : %s", data->name_bar_gauge1_f);
		ROS_DEBUG ("\tStart value bar 1: %d", data->start_value_bar_1);
		ROS_DEBUG ("\tEnd value bar 1: %d", data->end_value_bar_1);
		ROS_DEBUG ("\tGreen strip start value, bar 1: %d", data->green_strip_start_1);
		ROS_DEBUG ("\tYellow strip start value, bar 1: %d", data->yellow_strip_start_1);
		ROS_DEBUG ("\tRed strip start value, bar 1: %d", data->red_strip_start_1);

		// Bar gauge 2
		np.param<std::string> ("bg_name_bar_gauge2", name_bar_gauge2, std::string("BG2"));
		np.param<std::string> ("bg_unit_bar_gauge2", unit_bar_gauge2, std::string("[x]"));
		sprintf(data->name_bar_gauge2_f,
		        "<big>%s</big>\n<span foreground=\"orange\"><i>(%s)</i></span>",
		        name_bar_gauge2.c_str(), unit_bar_gauge2.c_str());
		np.param<int> ("bg_start_value_bar_2", data->start_value_bar_2, 100);
		np.param<int> ("bg_end_value_bar_2", data->end_value_bar_2, 0);
		np.param<int> ("bg_green_strip_start_2", data->green_strip_start_2, data->end_value_bar_2);
		np.param<int> ("bg_yellow_strip_start_2", data->yellow_strip_start_2,
		               data->start_value_bar_2);
		np.param<int> ("bg_red_strip_start_2", data->red_strip_start_2, data->start_value_bar_2);
		ROS_DEBUG ("\tBar Gauge 2 name : %s", data->name_bar_gauge2_f);
		ROS_DEBUG ("\tStart value bar 2: %d", data->start_value_bar_2);
		ROS_DEBUG ("\tEnd value bar 2: %d", data->end_value_bar_2);
		ROS_DEBUG ("\tGreen strip start value, bar 2: %d", data->green_strip_start_2);
		ROS_DEBUG ("\tYellow strip start value, bar 2: %d", data->yellow_strip_start_2);
		ROS_DEBUG ("\tRed strip start value, bar 2: %d", data->red_strip_start_2);

		// Bar gauge 3
		np.param<std::string> ("bg_name_bar_gauge3", name_bar_gauge3, std::string("BG3"));
		np.param<std::string> ("bg_unit_bar_gauge3", unit_bar_gauge3, std::string("[x]"));
		sprintf(data->name_bar_gauge3_f,
		        "<big>%s</big>\n<span foreground=\"orange\"><i>(%s)</i></span>",
		        name_bar_gauge3.c_str(), unit_bar_gauge3.c_str());
		np.param<int> ("bg_start_value_bar_3", data->start_value_bar_3, 100);
		np.param<int> ("bg_end_value_bar_3", data->end_value_bar_3, 0);
		np.param<int> ("bg_green_strip_start_3", data->green_strip_start_3, data->end_value_bar_3);
		np.param<int> ("bg_yellow_strip_start_3", data->yellow_strip_start_3,
		               data->start_value_bar_3);
		np.param<int> ("bg_red_strip_start_3", data->red_strip_start_3, data->start_value_bar_3);
		ROS_DEBUG ("\tBar Gauge 3 name : %s", data->name_bar_gauge3_f);
		ROS_DEBUG ("\tStart value bar 3: %d", data->start_value_bar_3);
		ROS_DEBUG ("\tEnd value bar 3: %d", data->end_value_bar_3);
		ROS_DEBUG ("\tGreen strip start value, bar 3: %d", data->green_strip_start_3);
		ROS_DEBUG ("\tYellow strip start value, bar 3: %d", data->yellow_strip_start_3);
		ROS_DEBUG ("\tRed strip start value, bar 3: %d", data->red_strip_start_3);

		// -----------------------------------------------------------------
		// **** allow widget creation
		data->ros_param_read = true;

		// **** wait to widget creation
		ROS_DEBUG ("Waiting widgets creation");
		while (!data->widget_created)
			usleep(25000);

		// -----------------------------------------------------------------
		// **** topics subscribing
		ROS_INFO ("Subscribing to topics");
		airframeStateSub = n.subscribe("/airframeState", 1, airframeStateCallback);
		systemStateSub = n.subscribe("/systemState", 1, systemStateCallback);
		gpsStateSub = n.subscribe("/gpsData", 1, gpsStateCallback);

		ROS_INFO ("Spinning");
		ros::spin();
	}

	// **** stop the gtk main loop
	if (GTK_IS_WIDGET (data->window)) {
		gtk_main_quit();
	}
	pthread_exit(NULL);
}

/**
 * @fn gboolean widgets_update(gpointer dat)
 * @brief Gtk function which allow to refresh the widget_table.<br>
 * This allow the child widgets to be redraw also.<br>
 */
gboolean widgets_update(gpointer dat) {
	gtk_widget_draw(GTK_WIDGET (data->widget_table), NULL);
	return true;
}

void load_icon() {
	GtkWidget *img_record, *img_record_g, *img_stop, *img_pause, *img_refresh;
	GtkWidget *img_leftarrow, *img_rightarrow, *img_status_ok, *img_status_fail;

	char icon_file[FILENAME_MAX];
	sprintf(icon_file, "%s/%s", data->icon_directory, "record-64.png");
	img_record = gtk_image_new_from_file(icon_file);
	data->record_icon_64 = gtk_image_get_pixbuf(GTK_IMAGE (img_record));

	sprintf(icon_file, "%s/%s", data->icon_directory, "record_g-64.png");
	img_record_g = gtk_image_new_from_file(icon_file);
	data->record_g_icon_64 = gtk_image_get_pixbuf(GTK_IMAGE (img_record_g));

	sprintf(icon_file, "%s/%s", data->icon_directory, "pause-64.png");
	img_pause = gtk_image_new_from_file(icon_file);
	data->pause_icon_64 = gtk_image_get_pixbuf(GTK_IMAGE (img_pause));

	sprintf(icon_file, "%s/%s", data->icon_directory, "stop-64.png");
	img_stop = gtk_image_new_from_file(icon_file);
	data->stop_icon_64 = gtk_image_get_pixbuf(GTK_IMAGE (img_stop));

	sprintf(icon_file, "%s/%s", data->icon_directory, "refresh-64.png");
	img_refresh = gtk_image_new_from_file(icon_file);
	data->refresh_icon_64 = gtk_image_get_pixbuf(GTK_IMAGE (img_refresh));

	sprintf(icon_file, "%s/%s", data->icon_directory, "rightarrow-64.png");
	img_rightarrow = gtk_image_new_from_file(icon_file);
	data->rightarrow_icon_64 = gtk_image_get_pixbuf(GTK_IMAGE (img_rightarrow));

	sprintf(icon_file, "%s/%s", data->icon_directory, "leftarrow-64.png");
	img_leftarrow = gtk_image_new_from_file(icon_file);
	data->leftarrow_icon_64 = gtk_image_get_pixbuf(GTK_IMAGE (img_leftarrow));

	sprintf(icon_file, "%s/%s", data->icon_directory, "status-ok-64.png");
	img_status_ok = gtk_image_new_from_file(icon_file);
	data->status_ok_icon_64 = gtk_image_get_pixbuf(GTK_IMAGE (img_status_ok));

	sprintf(icon_file, "%s/%s", data->icon_directory, "status-fail-64.png");
	img_status_fail = gtk_image_new_from_file(icon_file);
	data->status_fail_icon_64 = gtk_image_get_pixbuf(GTK_IMAGE (img_status_fail));

	data->status_ok_icon_motor = GTK_WIDGET (gtk_image_new_from_pixbuf
			(gdk_pixbuf_scale_simple (data->status_ok_icon_64, 22, 22, GDK_INTERP_HYPER)));
	data->status_fail_icon_motor = GTK_WIDGET (gtk_image_new_from_pixbuf
			(gdk_pixbuf_scale_simple (data->status_fail_icon_64, 22, 22, GDK_INTERP_HYPER)));
	data->status_ok_icon_gps = GTK_WIDGET (gtk_image_new_from_pixbuf
			(gdk_pixbuf_scale_simple (data->status_ok_icon_64, 22, 22, GDK_INTERP_HYPER)));
	data->status_fail_icon_gps = GTK_WIDGET (gtk_image_new_from_pixbuf
			(gdk_pixbuf_scale_simple (data->status_fail_icon_64, 22, 22, GDK_INTERP_HYPER)));
	data->status_ok_icon_flying = GTK_WIDGET (gtk_image_new_from_pixbuf
			(gdk_pixbuf_scale_simple (data->status_ok_icon_64, 22, 22, GDK_INTERP_HYPER)));
	data->status_fail_icon_flying = GTK_WIDGET (gtk_image_new_from_pixbuf
			(gdk_pixbuf_scale_simple (data->status_fail_icon_64, 22, 22, GDK_INTERP_HYPER)));
	data->record_icon
	        = GTK_WIDGET (gtk_image_new_from_pixbuf (gdk_pixbuf_scale_simple (data->record_icon_64, 30, 30, GDK_INTERP_HYPER)));
	data->record_g_icon
	        = GTK_WIDGET (gtk_image_new_from_pixbuf (gdk_pixbuf_scale_simple (data->record_g_icon_64, 30, 30, GDK_INTERP_HYPER)));
}

/**
 * @fn int main (int argc, char **argv)
 * @brief Main program & Gtk thread.
 * 
 * Create window and all widgets, then set there parameters to be the <br>
 * ROS params.
 */
int main(int argc, char **argv) {
	GtkBuilder *builder;
	GdkColor black = { 0, 0, 0, 0 };
	GError *error = NULL;
	char glade_gui_file[FILENAME_MAX];
	int start_zoom = 15;
	char *mapcachedir;

	OsmGpsMapPoint ccny_coord = { 55.374014, 10.398442 };

	struct arg param;
	param.argc = argc;
	param.argv = argv;

	pthread_t rosThread;

	// **** init threads
	g_thread_init(NULL);
	gdk_threads_init();
	gdk_threads_enter();

	// **** init gtk
	gtk_init(&argc, &argv);

	// **** allocate data structure
	data = g_slice_new (AppData);

	// **** set the glade gui file & set icon directory
	std::string package_path = ros::package::getPath(ROS_PACKAGE_NAME);
	sprintf(glade_gui_file, "%s/gui/%s", package_path.c_str(), "gui.glade");
	sprintf(data->icon_directory, "%s/gui/icon", package_path.c_str());

	std::string rosbag_path = ros::package::getPath("rosbag");
	sprintf(data->rosbag_rec_path, "%s/bin/record", rosbag_path.c_str());

	data->current_page = 0;
	data->telemetry_opt_popup_state = false;
	data->gps_opt_popup_state = false;
	data->fullscreen = false;
	load_icon();

	// **** Create new GtkBuilder object
	builder = gtk_builder_new();
	// **** Load UI from file
	if (!gtk_builder_add_from_file(builder, glade_gui_file, &error)) {
		g_warning ("%s", error->message);
		g_free(error);
		exit(-1);
	}

	// **** Get main window pointer from UI
	data->window = GTK_WIDGET (gtk_builder_get_object (builder, "window1"));
	gtk_window_set_title(GTK_WINDOW (data->window), "CityFlyer Ground Station");
	gtk_window_set_position(GTK_WINDOW (data->window), GTK_WIN_POS_CENTER);
	gtk_window_set_default_size(GTK_WINDOW (data->window), 1024, 576);

	// **** create ROS thread
	pthread_create(&rosThread, NULL, startROS, &param);

	// **** wait ros finish read params
	ROS_INFO("Waiting for ROS parameters...");
	while (!data->ros_param_read)
		usleep(25000);

	// **** Get GtkNotebook objsect
	data->notebook = GTK_WIDGET (gtk_builder_get_object (builder, "notebook1"));

	// #####################################################################
	// #####################################################################
	// **** Tab 1: Telemetry

	// **** create gauge1 widget (Airspeed)

	data->gauge1 = gtk_gauge_new();
	g_object_set(GTK_GAUGE (data->gauge1), "name", data->gauge1_name_f, NULL);
	g_object_set(GTK_GAUGE (data->gauge1), "grayscale-color", data->grayscale_color,
	             "radial-color", data->radial_color, "start-value", data->gauge1_start_value,
	             "end-value", data->gauge1_end_value, "initial-step", data->gauge1_initial_step,
	             "sub-step", (gdouble) data->gauge1_sub_step, "drawing-step",
	             data->gauge1_drawing_step, "color-strip-order", data->gauge1_color_strip_order,
	             "green-strip-start", data->gauge1_green_strip_start, "yellow-strip-start",
	             data->gauge1_yellow_strip_start, "orange-strip-start",
	             data->gauge1_orange_strip_start, "red-strip-start", data->gauge1_red_strip_start,
	             NULL);

	// **** create artificial horizon widgets
	data->arh = gtk_artificial_horizon_new();
	g_object_set(GTK_ARTIFICIAL_HORIZON (data->arh), "grayscale-color", data->grayscale_color,
	             "radial-color", data->radial_color, NULL);

	// **** create altimeter widgets
	data->alt = gtk_altimeter_new();
	g_object_set(GTK_ALTIMETER (data->alt), "grayscale-color", data->grayscale_color,
	             "unit-is-feet", data->altimeter_unit_is_feet, "unit-step-value",
	             data->altimeter_step_value, "radial-color", data->radial_color, NULL);

	// **** create turn coordinater widget
	data->tc = gtk_turn_coordinator_new();
	g_object_set(GTK_TURN_COORDINATOR (data->tc), "grayscale-color", data->grayscale_color,
	             "radial-color", data->radial_color, NULL);

	// **** create compass widgets
	data->comp = gtk_compass_new();
	g_object_set(GTK_COMPASS (data->comp), "grayscale-color", data->grayscale_color,
	             "radial-color", data->radial_color, NULL);

	// **** create gauge widgets
	data->bg = gtk_bar_gauge_new();
	g_object_set(GTK_BAR_GAUGE (data->bg), "widget-name", data->widget_name, NULL);
	g_object_set(GTK_BAR_GAUGE (data->bg), "name-bar-1", data->name_bar_gauge1_f, NULL);
	g_object_set(GTK_BAR_GAUGE (data->bg), "name-bar-2", data->name_bar_gauge2_f, NULL);
	g_object_set(GTK_BAR_GAUGE (data->bg), "name-bar-3", data->name_bar_gauge3_f, NULL);
	g_object_set(GTK_BAR_GAUGE (data->bg), "bar-number", data->bar_number, "grayscale-color",
	             data->grayscale_color, "radial-color", data->radial_color, "start-value-bar-1",
	             data->start_value_bar_1, "end-value-bar-1",
	             data->end_value_bar_1,
	             "green-strip-start-1",
	             data->green_strip_start_1,
	             "yellow-strip-start-1",
	             data->yellow_strip_start_1,
	             //										   "red-strip-start-1", data->red_strip_start_1,
	             "start-value-bar-2", data->start_value_bar_2, "end-value-bar-2",
	             data->end_value_bar_2, "green-strip-start-2", data->green_strip_start_2,
	             "yellow-strip-start-2",
	             data->yellow_strip_start_2,
	             //										   "red-strip-start-2", data->red_strip_start_2,
	             "start-value-bar-3", data->start_value_bar_3, "end-value-bar-3",
	             data->end_value_bar_3, "green-strip-start-3", data->green_strip_start_3,
	             "yellow-strip-start-3", data->yellow_strip_start_3,
	             //										   "red-strip-start-3", data->red_strip_start_3,
	             NULL);

	data->widget_table = GTK_WIDGET (gtk_builder_get_object (builder, "table_Widgets"));
	gtk_table_attach_defaults(GTK_TABLE (data->widget_table), data->gauge1, 0, 1, 0, 1); /* Airspeed indicator */
	gtk_table_attach_defaults(GTK_TABLE (data->widget_table), data->arh, 1, 2, 0, 1); /* Artificial horizon */
	gtk_table_attach_defaults(GTK_TABLE (data->widget_table), data->alt, 2, 3, 0, 1); /* Alti-meter */
	gtk_table_attach_defaults(GTK_TABLE (data->widget_table), data->tc, 0, 1, 1, 2); /* Turn coordinator */
	gtk_table_attach_defaults(GTK_TABLE (data->widget_table), data->comp, 1, 2, 1, 2); /* Compass */
	gtk_table_attach_defaults(GTK_TABLE (data->widget_table), data->bg, 2, 3, 1, 2); /* bar gauges */

	gtk_widget_modify_bg(data->gauge1, GTK_STATE_NORMAL, &black);
	gtk_widget_modify_bg(data->arh, GTK_STATE_NORMAL, &black);
	gtk_widget_modify_bg(data->alt, GTK_STATE_NORMAL, &black);
	gtk_widget_modify_bg(data->tc, GTK_STATE_NORMAL, &black);
	gtk_widget_modify_bg(data->comp, GTK_STATE_NORMAL, &black);
	gtk_widget_modify_bg(data->bg, GTK_STATE_NORMAL, &black);

	data->telemetry_option_popup
	        = GTK_WIDGET (gtk_builder_get_object (builder, "hbox_TelemetryOption"));
	data->btn_open_telemetry_option_popup
	        = GTK_WIDGET (gtk_builder_get_object (builder, "button_OpenTelemetryOptionPopup"));
	data->btn_close_telemetry_option_popup
	        = GTK_WIDGET (gtk_builder_get_object (builder, "button_CloseTelemetryOptionPopup"));

	gtk_button_set_image(
	                     GTK_BUTTON (data->btn_open_telemetry_option_popup),
	                     gtk_image_new_from_pixbuf(
	                                               gdk_pixbuf_scale_simple(data->leftarrow_icon_64,
	                                                                       24, 50, GDK_INTERP_HYPER)));
	gtk_button_set_image(
	                     GTK_BUTTON (data->btn_close_telemetry_option_popup),
	                     gtk_image_new_from_pixbuf(
	                                               gdk_pixbuf_scale_simple(
	                                                                       data->rightarrow_icon_64,
	                                                                       24, 50, GDK_INTERP_HYPER)));

	// #####################################################################
	// #####################################################################
	// **** Tab 2: Gps

	// Some GpsdViewer initialisation
	data->draw_path = false;
	data->map_provider = OSM_GPS_MAP_SOURCE_OPENSTREETMAP;
	data->map_zoom_max = 18;
	data->map_current_zoom = start_zoom;
	data->repo_uri = osm_gps_map_source_get_repo_uri(data->map_provider);
	data->friendly_name = osm_gps_map_source_get_friendly_name(data->map_provider);
	data->uav_track = osm_gps_map_track_new();
	mapcachedir = osm_gps_map_get_default_cache_directory();
	data->cachedir = g_build_filename(mapcachedir, data->friendly_name, NULL);
	g_free(mapcachedir);

	// Create the OsmGpsMap object
	data->map = (OsmGpsMap *) g_object_new(OSM_TYPE_GPS_MAP, "map-source", data->map_provider,
	                                       "tile-cache", data->cachedir, "proxy-uri",
	                                       g_getenv("http_proxy"), NULL);

	//Set the starting coordinates and zoom level for the map
	osm_gps_map_set_zoom(data->map, start_zoom);
	osm_gps_map_set_center(data->map, ccny_coord.rlat, ccny_coord.rlon);

	data->osd = gpsd_viewer_osd_new();
	g_object_set(GPSD_VIEWER_OSD (data->osd), "show-scale", true, "show-coordinates", true,
	             "show-dpad", true, "show-zoom", true, "show-gps-in-dpad", true,
	             "show-gps-in-zoom", false, "dpad-radius", 30, NULL);
	osm_gps_map_layer_add(OSM_GPS_MAP (data->map), OSM_GPS_MAP_LAYER (data->osd));

	data->map_box = GTK_WIDGET (gtk_builder_get_object (builder, "hbox_map_box"));
	data->map_container = GTK_WIDGET (gtk_builder_get_object (builder, "hbox_map_container"));
	gtk_box_pack_start(GTK_BOX (data->map_box), GTK_WIDGET (data->map), TRUE, TRUE, 0);

	data->gpsd_option_popup = GTK_WIDGET (gtk_builder_get_object (builder, "hbox_GpsdOptionPopup"));
	data->btn_open_gpsd_option_popup
	        = GTK_WIDGET (gtk_builder_get_object (builder, "button_OpenGpsdOptionPopup"));
	data->btn_close_gpsd_option_popup
	        = GTK_WIDGET (gtk_builder_get_object (builder, "button_CloseGpsdOptionPopup"));
	gtk_button_set_image(
	                     GTK_BUTTON (data->btn_open_gpsd_option_popup),
	                     gtk_image_new_from_pixbuf(
	                                               gdk_pixbuf_scale_simple(data->leftarrow_icon_64,
	                                                                       24, 50, GDK_INTERP_HYPER)));
	gtk_button_set_image(
	                     GTK_BUTTON (data->btn_close_gpsd_option_popup),
	                     gtk_image_new_from_pixbuf(
	                                               gdk_pixbuf_scale_simple(
	                                                                       data->rightarrow_icon_64,
	                                                                       24, 50, GDK_INTERP_HYPER)));

	// #####################################################################
	// #####################################################################
	// **** Tab 3: Rec

	data->recording = 0;
	data->rosbag_record_cmd = "rosbag record";
	data->topicsList = GTK_LIST_STORE (gtk_builder_get_object (builder, "liststore_TopicList"));
	data->cmd_line_entry = GTK_WIDGET (gtk_builder_get_object (builder, "entry_CommandLine"));
	data->prefix_entry = GTK_WIDGET (gtk_builder_get_object (builder, "entry_Prefix"));
	data->info_textview = GTK_WIDGET (gtk_builder_get_object (builder, "textview_BagInfo"));
	data->update_btn = GTK_WIDGET (gtk_builder_get_object (builder, "button_UpdateTopicList"));
	data->box_MotorStatus = GTK_WIDGET (gtk_builder_get_object (builder, "hbox_MotorStatus"));
	data->box_Flying = GTK_WIDGET (gtk_builder_get_object (builder, "hbox_Flying"));
	data->box_Gps = GTK_WIDGET (gtk_builder_get_object (builder, "hbox_Gps"));
	data->flightMode_label = GTK_WIDGET (gtk_builder_get_object (builder, "label_FlightModeValue"));
	data->upTime_label = GTK_WIDGET (gtk_builder_get_object (builder, "label_UpTimeValue"));
	data->cpuLoad_label = GTK_WIDGET (gtk_builder_get_object (builder, "label_CpuLoadValue"));
	data->box_RecordStatus = GTK_WIDGET (gtk_builder_get_object (builder, "hbox_RecordStatus"));
	data->record_stop_btn = GTK_WIDGET (gtk_builder_get_object (builder, "button_RecordStop"));

	gtk_box_pack_end(GTK_BOX (data->box_MotorStatus), data->status_ok_icon_motor, TRUE, TRUE, 0);
	gtk_box_pack_end(GTK_BOX (data->box_MotorStatus), data->status_fail_icon_motor, TRUE, TRUE, 0);
	gtk_box_pack_end(GTK_BOX (data->box_Flying), data->status_ok_icon_flying, TRUE, TRUE, 0);
	gtk_box_pack_end(GTK_BOX (data->box_Flying), data->status_fail_icon_flying, TRUE, TRUE, 0);
	gtk_box_pack_end(GTK_BOX (data->box_Gps), data->status_ok_icon_gps, TRUE, TRUE, 0);
	gtk_box_pack_end(GTK_BOX (data->box_Gps), data->status_fail_icon_gps, TRUE, TRUE, 0);
	gtk_box_pack_end(GTK_BOX (data->box_RecordStatus), data->record_icon, TRUE, TRUE, 0);
	gtk_box_pack_end(GTK_BOX (data->box_RecordStatus), data->record_g_icon, TRUE, TRUE, 0);

	gtk_button_set_image(
	                     GTK_BUTTON (data->update_btn),
	                     gtk_image_new_from_pixbuf(
	                                               gdk_pixbuf_scale_simple(data->refresh_icon_64,
	                                                                       24, 24, GDK_INTERP_HYPER)));
	gtk_button_set_image(
	                     GTK_BUTTON (data->record_stop_btn),
	                     gtk_image_new_from_pixbuf(
	                                               gdk_pixbuf_scale_simple(data->record_icon_64,
	                                                                       40, 40, GDK_INTERP_HYPER)));

	// Connect signals
	gtk_builder_connect_signals(builder, data);

	// Destroy builder, since we don't need it anymore
	g_object_unref(G_OBJECT (builder));

	// Show window. All other widgets are automatically shown by GtkBuilder
	gtk_widget_show_all(data->window);
	gtk_widget_hide(data->record_icon);
	gtk_widget_hide(data->status_ok_icon_motor);
	gtk_widget_hide(data->status_ok_icon_flying);
	gtk_widget_hide(data->status_ok_icon_gps);
	gtk_widget_hide_all(data->telemetry_option_popup);
	gtk_widget_hide_all(data->gpsd_option_popup);

	// **** allow ROS spinning
	data->widget_created = true;

	// **** udpate all widgets
	g_timeout_add(data->telemetry_refresh_rate, widgets_update, NULL);

	gtk_main();
	gdk_threads_leave();
	return 0;
}

void airframeStateCallback(const fmMsgs::airframeState::ConstPtr& msg) {
	gdk_threads_enter();
	/************* speedometer */
	double airspeed = msg->Va  * 3.6; // [m/s] -> [km/hr]
	airspeed = airspeed < 0 ? 0 : airspeed;
	if (IS_GTK_GAUGE (data->gauge1))
		gtk_gauge_set_value(GTK_GAUGE (data->gauge1), airspeed);

	/************* altimeter */
	double altitude = msg->alt;
	altitude = altitude < 0 ? 0 : altitude;

	if (IS_GTK_ALTIMETER (data->alt))
		gtk_altimeter_set_alti(GTK_ALTIMETER (data->alt), altitude);

	/************* artificial horizon */
	gdouble roll = -RAD2DEG(msg->pose.x);
	gdouble pitch = RAD2DEG(msg->pose.y) * 2;

	roll = roll < 0 ? roll + 360 : roll;
	roll = roll > 360 ? roll - 360 : roll;
	pitch = pitch > 70 ? 70 : pitch;
	pitch = pitch < -70 ? -70 : pitch;

	if (IS_GTK_ARTIFICIAL_HORIZON (data->arh))
		gtk_artificial_horizon_set_value(GTK_ARTIFICIAL_HORIZON (data->arh), roll, pitch);

	/************* Turn coordinator */
	gdouble incline = -msg->incline * 6;
	static gdouble ballp = RAD2DEG(incline), ballv=0;
	static const double ballMass = 0.005, ballDampening = 0.05;
	static const double pMax = 100;
	gdouble balla;
	static ros::Time t_;
	ros::Duration dt = ros::Time::now() - t_;
	t_ = ros::Time::now();

	if (incline > 0.01)
		incline =  sqrt(incline);
	else if(incline < -0.01)
		incline = -sqrt(-incline);


	balla = sin(incline - DEG2RAD(ballp)) * 9.82 / ballMass;

//	balla = (incline - ballp);
	ballv = ballv + dt.toSec() * balla - ballv * ballDampening;
	ballp = ballp + ballv * dt.toSec();

	if (ballp > pMax) {
		ballp = pMax;
		ballv = 0;
	}
	if (ballp < -pMax) {
		ballp = -pMax;
		ballv = 0;
	}

	roll = RAD2DEG(msg->pose.x);
	roll = roll < 0 ? roll + 360 : roll;
	roll = roll > 360 ? roll - 360 : roll;

	if (IS_GTK_TURN_COORDINATOR(data->tc))
		gtk_turn_coordinator_set_value(GTK_TURN_COORDINATOR (data->tc), roll, ballp);

	/************* compass */
	gdouble yaw = RAD2DEG(msg->pose.z);
	yaw = yaw > 360 ? yaw - 360 : yaw;
	yaw = yaw < 0 ? yaw + 360 : yaw;
	if (IS_GTK_COMPASS (data->comp))
		gtk_compass_set_angle(GTK_COMPASS (data->comp), yaw);
	gdk_threads_leave();
}

void gpsStateCallback(const fmMsgs::gps_state::ConstPtr& msg) {
	gdk_threads_enter();
	/************* GPS map */
	gint pixel_x, pixel_y;
	if (msg->fix > 0) { //GPS fix/noFix icon
		OsmGpsMapPoint *point = osm_gps_map_point_new_degrees(msg->lat, msg->lon); // lat, lon
		osm_gps_map_convert_geographic_to_screen(data->map, point, &pixel_x, &pixel_y);

		if (OSM_IS_GPS_MAP (data->map)) {
			// **** Center map on gps data received
			if (data->lock_view) {
				update_uav_pose_osd(data->osd, TRUE, pixel_x, pixel_y);
				osm_gps_map_set_center(data->map, msg->lat, msg->lon); // lat, lon
			} else {
				update_uav_pose_osd(data->osd, FALSE, pixel_x, pixel_y);
				osm_gps_map_gps_clear(data->map);
			}

			// **** Add point to the track
			osm_gps_map_track_add_point(data->uav_track, point);
		}
	}
	if (IS_GTK_COMPASS (data->comp))
		gtk_compass_set_course(GTK_COMPASS (data->comp), msg->cogt);
	gdk_threads_leave();
}

void systemStateCallback(const fmMsgs::sysState::ConstPtr& msg) {
	gdk_threads_enter();
	/************* battery, cpu, mem */
	double cpu_load = (double) 100 - msg->cpuload;
	double memutil = (double) 100 - msg->memutil;
	double battery = (double) msg->battery;

	if (IS_GTK_BAR_GAUGE (data->bg)) {
		gtk_bar_gauge_set_value(GTK_BAR_GAUGE (data->bg), 1, battery);
		gtk_bar_gauge_set_value(GTK_BAR_GAUGE (data->bg), 2, cpu_load);
		gtk_bar_gauge_set_value(GTK_BAR_GAUGE (data->bg), 3, memutil);
	}

	gdk_threads_leave();
}
