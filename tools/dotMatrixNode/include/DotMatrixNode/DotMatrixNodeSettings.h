namespace DotMatrixNodeSettings {

	/**
	 * @var static const unsigned int DRAW_FIELD_HEIGHT
	 * Height of the field that is drawable by the deltarobot.
	 */
	static const unsigned int DRAW_FIELD_HEIGHT = 175;
	/**
	 * @var static const unsigned int DRAW_FIELD_WIDTH
	 * Width of the field that is drawable by the deltarobot.
	 */
	static const unsigned int DRAW_FIELD_WIDTH = 110;

	/**
	 * @var static const double DRAW_FIELD_DOTS_PER_MM
	 * Number of dots per millimeter on the drawing field.
	 */
	static const double DRAW_FIELD_DOTS_PER_MM = 1;
	/**
	 * @var static const double DRAW_FIELD_MM_PER_DOTS
	 * Number of millimeters per dot on the drawing field.
	 **/
	static const double DRAW_FIELD_MM_PER_DOTS = 1 / DRAW_FIELD_DOTS_PER_MM;

	/**
	 * @var static const double DRAW_FIELD_Z_LOW
	 * The z position where the pen reaches the paper.
	 **/
	static const double DRAW_FIELD_Z_LOW = 300;//TODO: ?
	/**
	 * @var static const double DRAW_FIELD_Z_HIGH
	 * The z position where the pen doesn't reach the paper, and is just safe for movement.
	 **/
	static const double DRAW_FIELD_Z_HIGH = 280;//TODO: ?

}
